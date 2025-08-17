#include "microros_bridge.h"
#include "plotter_config.h"

#include <string.h>
#include <inttypes.h>
#include "esp_log.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/empty.h>
#include <std_msgs/msg/u_int8.h>
#include <std_msgs/msg/u_int8_multi_array.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>
#include <rmw/qos_profiles.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// интеграция со слоями проекта
#include "ring_buffer.h"
#include "esp32_to_stm32.h"
#include "spi_feeder.h"

// ================== Константы протокола/алгоритма ==================
#ifndef RX_PACKET_BYTES
#define RX_PACKET_BYTES              480u     // размер «полезной» части пакета от паблишера
#endif

// целевая «почти-полная» заполненность кольцевого буфера
#ifndef SLACK_BYTES
#define SLACK_BYTES                  RX_PACKET_BYTES   // небольшой зазор под очередной пакет
#endif

#ifndef ASK_MAX_PER_SHOT
#define ASK_MAX_PER_SHOT             40u      // разовый «дозаряд» буфера
#endif

// вместимость приёмного ROS-сообщения (для prealloc в executor)
#ifndef RX_MSG_CAP_BYTES
  #if SEQ_ID_ENABLED
    #define RX_MSG_CAP_BYTES (RX_PACKET_BYTES + 1)
  #else
    #define RX_MSG_CAP_BYTES (RX_PACKET_BYTES)
  #endif
#endif
// ==================================================================

static const char *TAG = "microros_bridge";

// micro-ROS объекты
static rcl_allocator_t      allocator;
static rclc_support_t       support;
static rcl_node_t           node;
static rclc_executor_t      executor;

static rcl_subscription_t   sub_stream;      // /plotter/byte_stream
static rcl_subscription_t   sub_draw_start;  // /plotter/cmd/draw_start
static rcl_subscription_t   sub_draw_finish; // /plotter/cmd/draw_finish
static rcl_publisher_t      pub_need;        // /plotter/cmd/need_packets  (RELIABLE)

// заранее выделенные сообщения
static std_msgs__msg__UInt8MultiArray s_msg_stream;
static std_msgs__msg__Empty           s_msg_empty;
static std_msgs__msg__UInt8           s_msg_need;

// состояние
static rb_t *s_rb = NULL;
static microros_bridge_stats_t s_stats = {0};

static volatile bool     s_draw_active       = false;
static volatile bool     s_finish_requested  = false;
static volatile bool     s_finish_cmd_sent   = false;

static volatile uint8_t  s_outstanding_pkts  = 0; // сколько запросили, но ещё не пришло

// ===== утилиты =====
#define RCCHECK(fn) do{ rcl_ret_t rc__ = (fn); if (rc__ != RCL_RET_OK){ \
    ESP_LOGE(TAG, "RCL err %d at %s:%d", (int)rc__, __FILE__, __LINE__); return; } }while(0)

static inline uint8_t clamp_u8(size_t v){ return (v > 255) ? 255u : (uint8_t)v; }

// сколько пакетов нужно добрать, чтобы держать буфер ≈ полный (RB_SIZE_BYTES - SLACK_BYTES)
static uint8_t pkts_needed_to_target(size_t used_bytes, uint8_t outstanding_pkts)
{
    const size_t target = (RB_SIZE_BYTES > SLACK_BYTES) ? (RB_SIZE_BYTES - SLACK_BYTES) : RB_SIZE_BYTES;
    size_t have = used_bytes + (size_t)outstanding_pkts * (size_t)RX_PACKET_BYTES;
    if (have >= target) return 0;

    size_t deficit = target - have;
    size_t need    = (deficit + RX_PACKET_BYTES - 1) / RX_PACKET_BYTES; // ceil
    if (need > ASK_MAX_PER_SHOT) need = ASK_MAX_PER_SHOT;
    return clamp_u8(need);
}

static void publish_need_packets(uint8_t n){
    if (n == 0) return;
    s_msg_need.data = n;
    rcl_ret_t rc = rcl_publish(&pub_need, &s_msg_need, NULL);
    if (rc == RCL_RET_OK){
        s_outstanding_pkts = (uint8_t)(s_outstanding_pkts + n);
        ESP_LOGI(TAG, "need_packets: %u (outstanding=%u, rb_used=%u/%u)",
                 n, s_outstanding_pkts, (unsigned)rb_used(s_rb), (unsigned)RB_SIZE_BYTES);
    } else {
        ESP_LOGW(TAG, "need_packets publish failed rc=%d", (int)rc);
    }
}

// ===== callbacks =====
static void sub_stream_cb(const void * msgin)
{
    const std_msgs__msg__UInt8MultiArray *m = (const std_msgs__msg__UInt8MultiArray*)msgin;
    const uint8_t *data = m->data.data;
    size_t len = m->data.size;

#if SEQ_ID_ENABLED
    if (len < 1){
        s_stats.drops_total++;
        return;
    }
    uint8_t seq = data[0];
    if (s_stats.last_seq >= 0){
        uint8_t expect = ((uint8_t)s_stats.last_seq + 1u) & 0xFF;
        if (seq != expect){
            s_stats.seq_missed++;
            ESP_LOGW(TAG, "seq miss: expect=%u got=%u", expect, seq);
        }
    }
    s_stats.last_seq = seq;
    data += 1;
    len  -= 1;
#endif

    if (len == 0){
        s_stats.drops_total++;
        return;
    }
    if (len != RX_PACKET_BYTES){
        ESP_LOGW(TAG, "unexpected payload len=%u (expected %u)",
                 (unsigned)len, (unsigned)RX_PACKET_BYTES);
    }

    size_t w = rb_write_all_or_drop(s_rb, data, len);
    if (w == 0){
        s_stats.drops_total++;
        ESP_LOGW(TAG, "RB full: drop %u", (unsigned)len);
        return;
    }

    if (s_outstanding_pkts > 0) s_outstanding_pkts--;

    s_stats.pkts_in++;
    s_stats.bytes_in += (uint32_t)w;
}

static void sub_draw_start_cb(const void * msgin)
{
    (void)msgin;

    // сброс локального состояния/статистики
    s_stats               = (microros_bridge_stats_t){0};
    s_stats.last_seq      = -1;
    s_outstanding_pkts    = 0;
    s_finish_requested    = false;
    s_finish_cmd_sent     = false;

    rb_reset(s_rb);

    // уведомляем STM32 о начале приёма стрима
    plotter_send_cmd(CMD_DRAW_BEGIN, NULL);
    s_draw_active = true;

    // стартовая догрузка — сразу стремимся заполнить RB почти под завязку
    {
        size_t used = rb_used(s_rb);
        size_t free_pkts = rb_free_space(s_rb) / RX_PACKET_BYTES;
        uint8_t ask = pkts_needed_to_target(used, 0);
        if (ask > free_pkts) ask = clamp_u8(free_pkts);
        if (ask) publish_need_packets(ask);
    }

    ESP_LOGI(TAG, "DRAW_START");
}

static void sub_draw_finish_cb(const void * msgin)
{
    (void)msgin;
    s_finish_requested = true;
    ESP_LOGI(TAG, "DRAW_FINISH requested");
}

// ===== задачи =====
static void need_request_task(void *arg)
{
    (void)arg;
    for(;;){
        if (s_draw_active){
            if (!s_finish_requested){
                // публикуем ДОБОР только когда пред. запрос уже «закрыт»
                if (s_outstanding_pkts == 0){
                    size_t used      = rb_used(s_rb);
                    size_t free_pkts = rb_free_space(s_rb) / RX_PACKET_BYTES;

                    uint8_t need = pkts_needed_to_target(used, 0);
                    if (need > free_pkts) need = clamp_u8(free_pkts);
                    if (need > 0){
                        publish_need_packets(need);
                    }
                }
            } else {
                // Завершение: дожать остаток через фидер и отдать команду STM32
                if (s_outstanding_pkts == 0 && !spi_feeder_is_flushed()){
                    spi_feeder_request_finish();
                }
                if (s_outstanding_pkts == 0 && spi_feeder_is_flushed() && !s_finish_cmd_sent){
                    plotter_send_cmd(CMD_DRAW_FINISH_WHEN_EMPTY, NULL);
                    s_finish_cmd_sent = true;
                    s_draw_active     = false;
                    ESP_LOGI(TAG, "DRAW_FINISH sent to STM32 (after flush)");
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

static void log_task(void *arg)
{
    (void)arg;
    for(;;){
        ESP_LOGI(TAG,
                 "in: pkts=%" PRIu32 " bytes=%" PRIu32 " drops=%" PRIu32
                 " seqmiss=%" PRIu32 " rb_used=%u/%u outstanding=%u active=%d finish=%d flushed=%d",
                 s_stats.pkts_in,
                 s_stats.bytes_in,
                 s_stats.drops_total,
                 s_stats.seq_missed,
                 (unsigned)rb_used(s_rb),
                 (unsigned)RB_SIZE_BYTES,
                 (unsigned)s_outstanding_pkts,
                 (int)s_draw_active,
                 (int)s_finish_requested,
                 (int)spi_feeder_is_flushed());
        vTaskDelay(pdMS_TO_TICKS(LOG_PERIOD_MS));
    }
}

static void executor_task(void *arg)
{
    (void)arg;
    for(;;){
        (void) rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// ===== инициализация micro-ROS =====
static void microros_init_task(void *arg)
{
    rb_t *rb = (rb_t *)arg;
    s_rb = rb;

    vTaskDelay(pdMS_TO_TICKS(1000)); // небольшой стартовый лаг сети

    allocator = rcl_get_default_allocator();

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    {
        rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
        RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP,
                                                 CONFIG_MICRO_ROS_AGENT_PORT,
                                                 rmw_options));

        ESP_LOGI(TAG, "Ping agent %s:%s ...", CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT);
        while (rmw_uros_ping_agent(1000, 1) != RMW_RET_OK) {
            ESP_LOGW(TAG, "Agent not available, retrying...");
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
#endif

    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
    RCCHECK(rclc_node_init_default(&node, "esp32_plotter_node", "", &support));

    // Publisher: need_packets — ЯВНО RELIABLE
    RCCHECK(rclc_publisher_init(
        &pub_need, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
        "/plotter/cmd/need_packets",
        &rmw_qos_profile_default));  // reliable

    // Subscriber: stream (BestEffort, KeepLast(1))
    RCCHECK(rclc_subscription_init_best_effort(
        &sub_stream, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8MultiArray),
        "/plotter/byte_stream"));

    // Control subscribers: draw_start / draw_finish (RELIABLE по умолчанию)
    RCCHECK(rclc_subscription_init_default(
        &sub_draw_start, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty),
        "/plotter/cmd/draw_start"));

    RCCHECK(rclc_subscription_init_default(
        &sub_draw_finish, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty),
        "/plotter/cmd/draw_finish"));

    // Приёмные сообщения
    std_msgs__msg__UInt8MultiArray__init(&s_msg_stream);
    bool ok = rosidl_runtime_c__uint8__Sequence__init(&s_msg_stream.data, RX_MSG_CAP_BYTES);
    if (!ok){
        ESP_LOGE(TAG, "Sequence init failed");
        return;
    }
    std_msgs__msg__Empty__init(&s_msg_empty);
    std_msgs__msg__UInt8__init(&s_msg_need);

    // Executor + задачи
    RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &sub_stream,     &s_msg_stream,  &sub_stream_cb,     ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &sub_draw_start, &s_msg_empty,   &sub_draw_start_cb, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &sub_draw_finish,&s_msg_empty,   &sub_draw_finish_cb,ON_NEW_DATA));

    xTaskCreatePinnedToCore(executor_task,    "mr_executor",  9216, rb, 6, NULL, 0);
    xTaskCreatePinnedToCore(need_request_task,"mr_need_req",  6144, rb, 5, NULL, 0);
    xTaskCreatePinnedToCore(log_task,         "mr_log",       4096, rb, 3, NULL, 0);

    ESP_LOGI(TAG, "initialized (RB=%uB, pkt=%uB, slack=%uB, seq_id=%d)",
             (unsigned)RB_SIZE_BYTES, (unsigned)RX_PACKET_BYTES, (unsigned)SLACK_BYTES, (int)SEQ_ID_ENABLED);
    vTaskDelete(NULL);
}

// ===== API =====
void microros_bridge_init(rb_t *rb)
{
    s_stats.last_seq = -1;
    xTaskCreatePinnedToCore(microros_init_task, "mr_init", 16384, rb, 5, NULL, 0);
}

const microros_bridge_stats_t* microros_bridge_stats(void)
{
    return &s_stats;
}
