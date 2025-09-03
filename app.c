// app.c
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/empty.h>
#include <std_msgs/msg/u_int8.h>
#include <std_msgs/msg/u_int8_multi_array.h>
#include <xyplotter_msgs/msg/plotter_telemetry.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>
#include <rmw/qos_profiles.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
  #include <rmw_microros/rmw_microros.h>
#endif

#include "esp32_to_stm32.h"
#include "shared/cmd_ids.h"   // SPI_CHUNK_SIZE, CMD ids

// ---------- utils ----------
#define TAG "app"

#define RCCHECK(fn) do { \
  rcl_ret_t _rc = (fn); \
  if (_rc != RCL_RET_OK) { \
    printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)_rc); \
    vTaskDelete(NULL); \
  } \
} while (0)

// ---------- micro-ROS objects ----------
static rcl_allocator_t      g_allocator;
static rclc_support_t       g_support;
static rcl_node_t           g_node;
static rclc_executor_t      g_executor;

static rcl_subscription_t   g_sub_stream;      // /plotter/byte_stream (RELIABLE)
static rcl_subscription_t   g_sub_draw_start;  // /plotter/cmd/draw_start
static rcl_subscription_t   g_sub_draw_finish; // /plotter/cmd/draw_finish
static rcl_publisher_t      g_pub_need;        // /plotter/cmd/need_packets
static rcl_subscription_t   g_sub_home;        // /plotter/cmd/home
static rcl_subscription_t   g_sub_calibrate;   // /plotter/cmd/calibrate

static rcl_timer_t          g_need_pub_timer;

static std_msgs__msg__UInt8MultiArray g_msg_stream;
static std_msgs__msg__Empty           g_msg_empty_start;
static std_msgs__msg__Empty           g_msg_empty_finish;
static std_msgs__msg__UInt8           g_msg_need;
static std_msgs__msg__Empty           g_msg_empty_home;
static std_msgs__msg__Empty           g_msg_empty_calibrate;

static rcl_publisher_t  g_pub_telem;      // /plotter/telemetry
static rcl_timer_t      g_telem_timer;    // 100ms
static xyplotter_msgs__msg__PlotterTelemetry g_msg_telem;
// ---------- stream/buffers ----------
static volatile bool s_draw_active = false;

static uint8_t  s_data_buf[2][SPI_CHUNK_SIZE];
static volatile uint8_t s_data_buf_full[2] = {0, 0};

static TaskHandle_t s_sender_task = NULL;
static SemaphoreHandle_t s_need_sem = NULL;

void IRAM_ATTR ready_isr(void *arg)
{
  BaseType_t hpw = pdFALSE;
  if (s_sender_task) {
    vTaskNotifyGiveFromISR(s_sender_task, &hpw);
    if (hpw) portYIELD_FROM_ISR();
  }
}

static void telem_timer_cb(rcl_timer_t *timer, int64_t last_call_time)
{
  (void)timer; (void)last_call_time;

  plotter_state_t ps;
  plotter_get_state(&ps);

  g_msg_telem.x_steps       = (int32_t)ps.x_pos;
  g_msg_telem.y_steps       = (int32_t)ps.y_pos;
  g_msg_telem.is_calibrated = ps.is_calibrated ? true : false;
  g_msg_telem.is_homed      = ps.is_homed ? true : false;

  printf("timer called with data to send: ps.is_calibrated = %s \n", ps.is_calibrated ? "true" : "false");

  (void) rcl_publish(&g_pub_telem, &g_msg_telem, NULL);
}

static void sub_stream_cb(const void *msgin)
{
  static uint8_t fill_idx = 0;

  if (!s_draw_active) return;

  if (s_data_buf_full[fill_idx]) {
    ESP_LOGW(TAG, "stream overflow, dropping chunk into buf %u", fill_idx);
    return;
  }

  const std_msgs__msg__UInt8MultiArray *m =
      (const std_msgs__msg__UInt8MultiArray *)msgin;

  if (m->data.size < SPI_CHUNK_SIZE) {
    ESP_LOGW(TAG, "unexpected chunk size: %u (< %u)", (unsigned)m->data.size, (unsigned)SPI_CHUNK_SIZE);
  }

  memcpy(s_data_buf[fill_idx], m->data.data, SPI_CHUNK_SIZE);
  s_data_buf_full[fill_idx] = 1;
  fill_idx ^= 1;
}

static void sub_home_cb(const void *msgin)
{
  (void)msgin;
  printf("CMD/HOME message received");
  plotter_send_cmd(CMD_HOME, NULL);
}

static void sub_calibrate_cb(const void *msgin)
{
  (void)msgin;
  printf("CMD/CALIBRATE message received");
  plotter_send_cmd(CMD_CALIBRATE, NULL);
}

static void sub_draw_start_cb(const void *msgin)
{
  (void)msgin;
  if (s_draw_active) return;

  plotter_send_cmd(CMD_DRAW_BEGIN, NULL);
  s_draw_active = true;

  g_msg_need.data = 1;
  (void) rcl_publish(&g_pub_need, &g_msg_need, NULL);
}

static void sub_draw_finish_cb(const void *msgin)
{
  (void)msgin;
  s_draw_active = false;
  s_data_buf_full[0] = s_data_buf_full[1] = 0;
}

static void send_data_to_plotter_task(void *arg)
{
  (void)arg;
  s_sender_task = xTaskGetCurrentTaskHandle();

  uint8_t send_idx = 0;

  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    if (!s_draw_active) continue;

    while (!s_data_buf_full[send_idx]) {
      vTaskDelay(pdMS_TO_TICKS(1));
      if (!s_draw_active) break;
    }
    if (!s_draw_active) continue;

    plotter_send_draw_stream_data(s_data_buf[send_idx], SPI_CHUNK_SIZE);
    s_data_buf_full[send_idx] = 0;
    send_idx ^= 1;

    xSemaphoreGive(s_need_sem);
  }
}

static void need_pub_timer_cb(rcl_timer_t *timer, int64_t last_call_time)
{
  (void)timer; (void)last_call_time;

  while (xSemaphoreTake(s_need_sem, 0) == pdTRUE) {
    g_msg_need.data = 1;
    (void) rcl_publish(&g_pub_need, &g_msg_need, NULL);
  }
}

// ---------- main ----------
void appMain(void)
{
  plotter_init_sync(ready_isr);
  plotter_start_all_tasks();      // UART rx, control/heartbeat, keypad
  vTaskDelay(pdMS_TO_TICKS(200));

  g_allocator = rcl_get_default_allocator();

  rcl_init_options_t init_opts = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_opts, g_allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
  {
    rmw_init_options_t *rmw_opts = rcl_init_options_get_rmw_init_options(&init_opts);
    RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP,
                                             CONFIG_MICRO_ROS_AGENT_PORT,
                                             rmw_opts));
    // По желанию можно пинговать агента (без блокировок надолго)
    while (rmw_uros_ping_agent(300, 1) != RMW_RET_OK) {
      ESP_LOGW(TAG, "micro-ROS agent not available, retry...");
      vTaskDelay(pdMS_TO_TICKS(300));
    }
  }
#endif

  RCCHECK(rclc_support_init_with_options(&g_support, 0, NULL, &init_opts, &g_allocator));
  RCCHECK(rclc_node_init_default(&g_node, "esp32_plotter_node", "", &g_support));

  RCCHECK(rclc_publisher_init_default(
      &g_pub_need, &g_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
      "/plotter/cmd/need_packets"));


  rmw_qos_profile_t qos_telem = rmw_qos_profile_sensor_data; // BEST_EFFORT, KEEP_LAST, low-latency

  RCCHECK(rclc_publisher_init(
      &g_pub_telem, &g_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(xyplotter_msgs, msg, PlotterTelemetry),
      "/plotter/telemetry", &qos_telem));

  xyplotter_msgs__msg__PlotterTelemetry__init(&g_msg_telem);      

  RCCHECK(rclc_timer_init_default(&g_telem_timer, &g_support,
                                  RCL_MS_TO_NS(1000), telem_timer_cb));  

  rmw_qos_profile_t qos_stream = rmw_qos_profile_default;
  qos_stream.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  qos_stream.history     = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  qos_stream.depth       = 1;

  RCCHECK(rclc_subscription_init(
      &g_sub_stream, &g_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8MultiArray),
      "/plotter/byte_stream", &qos_stream));

  RCCHECK(rclc_subscription_init_default(
      &g_sub_draw_start, &g_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty),
      "/plotter/cmd/draw_start"));

  RCCHECK(rclc_subscription_init_default(
      &g_sub_draw_finish, &g_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty),
      "/plotter/cmd/draw_finish"));

  RCCHECK(rclc_subscription_init_default(
      &g_sub_home, &g_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty),
      "/plotter/cmd/home"));

  RCCHECK(rclc_subscription_init_default(
      &g_sub_calibrate, &g_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty),
      "/plotter/cmd/calibrate"));

  std_msgs__msg__UInt8MultiArray__init(&g_msg_stream);
  (void) rosidl_runtime_c__uint8__Sequence__init(&g_msg_stream.data, SPI_CHUNK_SIZE);
  std_msgs__msg__Empty__init(&g_msg_empty_start);
  std_msgs__msg__Empty__init(&g_msg_empty_finish);
  std_msgs__msg__Empty__init(&g_msg_empty_home);
  std_msgs__msg__Empty__init(&g_msg_empty_calibrate);
  std_msgs__msg__UInt8__init(&g_msg_need);

  s_need_sem = xSemaphoreCreateCounting(64, 0);
  configASSERT(s_need_sem);

  RCCHECK(rclc_timer_init_default(&g_need_pub_timer, &g_support,
                                  RCL_MS_TO_NS(5), need_pub_timer_cb));

  RCCHECK(rclc_executor_init(&g_executor, &g_support.context, 7, &g_allocator));
  RCCHECK(rclc_executor_add_timer(&g_executor, &g_need_pub_timer));
  RCCHECK(rclc_executor_add_timer(&g_executor, &g_telem_timer));
  RCCHECK(rclc_executor_add_subscription(&g_executor, &g_sub_stream,     &g_msg_stream,  &sub_stream_cb,     ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&g_executor, &g_sub_draw_start, &g_msg_empty_start,   &sub_draw_start_cb, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&g_executor, &g_sub_draw_finish,&g_msg_empty_finish,   &sub_draw_finish_cb,ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&g_executor, &g_sub_home,       &g_msg_empty_home,      &sub_home_cb,       ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&g_executor, &g_sub_calibrate,  &g_msg_empty_calibrate, &sub_calibrate_cb,  ON_NEW_DATA));
  
  xTaskCreatePinnedToCore(send_data_to_plotter_task, "mr_sender", 4096*2, NULL, 7, NULL, 1);

  for (;;) {
    rclc_executor_spin_some(&g_executor, RCL_MS_TO_NS(10));
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
