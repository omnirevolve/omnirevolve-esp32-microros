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

#include "esp32_to_stm32.h"

// ===================== DEFINES =======================
#define PACKETS_NUMBER_FOR_FIRST_REQUEST 5
#define RX_PACKET_BYTES 480u

#define RCCHECK(fn) do{ rcl_ret_t rc__ = (fn); if (rc__ != RCL_RET_OK){ \
    ESP_LOGE(TAG, "RCL err %d at %s:%d", (int)rc__, __FILE__, __LINE__); return; } }while(0)

// ===================== LOG TAG =======================
static const char *TAG = "microros_bridge";

// ===================== PROTOTYPES =======================
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

static volatile bool     s_draw_active       = false;
static volatile bool     s_finish_requested  = false;
static volatile bool     s_finish_cmd_sent   = false;

static volatile uint8_t  s_outstanding_pkts  = 0; // сколько запросили, но ещё не пришло

static uint8_t data_buf[2][SPI_CHUNK_SIZE];
static size_t active_buf_len = 0;
static uint8_t active_buf_index = 0;
static uint8_t *output_buf = NULL;
static size_t output_buf_size = 0;
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t sender_task_handle = NULL;

// ===================== NEED PACKETS =======================

static void publish_need_packets(uint8_t n){
    s_msg_need.data = n;
    for (;;)
    {
        rcl_ret_t rc = rcl_publish(&pub_need, &s_msg_need, NULL);
        if (rc == RCL_RET_OK){
            s_outstanding_pkts = (uint8_t)(s_outstanding_pkts + n);
            ESP_LOGI(TAG, "SENDING: need_packets: %u", n);
            break;
        } else {
            ESP_LOGW(TAG, "need_packets publish failed rc=%d", (int)rc);
        }
        ESP_LOGW(TAG, ">> Waiting for 20ms and repeat again");
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// ===================== PACKETS RECEIVING CALLBACK =======================
static void sub_stream_cb(const void * msgin)
{
    const std_msgs__msg__UInt8MultiArray *m = (const std_msgs__msg__UInt8MultiArray*)msgin;
    const uint8_t *data = m->data.data;
    size_t len = m->data.size;

    if (len == 0){
        ESP_LOGW(TAG, ">> Recieved len == 0");
        return;
    }
    
    if (len != RX_PACKET_BYTES){
        ESP_LOGW(TAG, ">> Unexpected payload len=%u (expected %u)",
                 (unsigned)len, (unsigned)RX_PACKET_BYTES);
    }

    if (!s_draw_active) {
        ESP_LOGW(TAG, "Received data from plotter data publisher when draw_active == FALSE");
        return;
    }

    if (s_draw_active && s_finish_requested && output_buf != NULL) {
        ESP_LOGW(TAG, "Check logic"); // maybe wrong logic
        return;
    }

    // check if we need write all the data in current buf:
    if (len + active_buf_len >= SPI_CHUNK_SIZE) {
        size_t extra_len = len + active_buf_len - SPI_CHUNK_SIZE;
        memcpy(data_buf[active_buf_index] + active_buf_len, data, len - extra_len);
        if (output_buf != NULL) {
            ESP_LOGW(TAG, ">> Has issue with buffer reader");
        }
        portENTER_CRITICAL(&s_mux);
        output_buf = data_buf[active_buf_index]; // set fulfilled buffer as output (to send to plotter)
        output_buf_size = SPI_CHUNK_SIZE;
        portEXIT_CRITICAL(&s_mux);
        xTaskNotifyGive(sender_task_handle);
        active_buf_index = !active_buf_index; // set another buf as active
        if (extra_len != 0) { // copy the rest of data if presented:
            memcpy(data_buf[active_buf_index], data + len - extra_len, extra_len);
        }
        active_buf_len = extra_len;
        publish_need_packets((SPI_CHUNK_SIZE - extra_len)/RX_PACKET_BYTES);
    }
    else {
        memcpy(data_buf[active_buf_index] + active_buf_len, data, len);
    }
    if (s_draw_active && s_finish_requested) {
        portENTER_CRITICAL(&s_mux);
        output_buf = data_buf[active_buf_index]; // set what we have as output (to send to plotter)
        output_buf_size = active_buf_len;
        portEXIT_CRITICAL(&s_mux);
    }
}

// ===================== DATA SENDER TO PLOTTER =======================
static void send_data_to_plotter_task(void *arg)
{
    (void)arg;
    
    ulTaskNotifyTake(pdTRUE, 0);
 
    for(;;){
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (!s_draw_active) {
            ESP_LOGW(TAG, ">> Sender was activated when there is no active drawing");
            continue;
        }
        
        // if we were waked up, then wait plotter ready signal:
        for(;;){
            if (plotter_is_ready_to_receive_draw_stream_data()) {
                if (output_buf_size < SPI_CHUNK_SIZE) {
                    if (!s_finish_requested){
                        ESP_LOGW(TAG, ">> Wrong logic in the code");
                    }
                    else {
                        ESP_LOGW(TAG, ">> Signal for the last packet of draw stream was received");
                    }
                    memset(output_buf + output_buf_size, 0, SPI_CHUNK_SIZE - output_buf_size);
                    output_buf_size = SPI_CHUNK_SIZE;
                }
                plotter_send_draw_stream_data(data_buf, output_buf_size);
                portENTER_CRITICAL(&s_mux);
                output_buf = NULL;
                output_buf_size = 0;
                portEXIT_CRITICAL(&s_mux);
            }
            else {
                vTaskDelay(pdMS_TO_TICKS(5));
            }
        }
    }   
}

// ===================== DRAW_START / DRAW_FINISH MESSAGE HANDLERS =======================

static void sub_draw_start_cb(const void * msgin)
{
    (void)msgin;
    plotter_send_cmd(CMD_DRAW_BEGIN, NULL);
    s_draw_active = true;
    publish_need_packets(PACKETS_NUMBER_FOR_FIRST_REQUEST);
}

static void sub_draw_finish_cb(const void * msgin)
{
    (void)msgin;
    s_finish_requested = true;
    ESP_LOGI(TAG, "DRAW_FINISH requested");
}

static void executor_task(void *arg)
{
    (void)arg;
    for(;;){
        (void) rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// ===================== MICRO-ROS INITIALIZING =======================
static void microros_init_task(void *arg)
{
    (void)arg;
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

    // Publisher: need_packets RELIABLE
    RCCHECK(rclc_publisher_init(
        &pub_need, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
        "/plotter/cmd/need_packets",
        &rmw_qos_profile_default));  // reliable

    // Subscriber: stream RELIABLE
    RCCHECK(rclc_subscription_init(
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

    std_msgs__msg__UInt8MultiArray__init(&s_msg_stream);
    bool ok = rosidl_runtime_c__uint8__Sequence__init(&s_msg_stream.data, RX_PACKET_BYTES);
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

    xTaskCreatePinnedToCore(executor_task,    "mr_executor",  9216, NULL, 6, NULL, 0);
    xTaskCreatePinnedToCore(send_data_to_plotter_task, "mr_sender", 2048, NULL, 5, &sender_task_handle, 0);

    vTaskDelete(NULL);
}

// ===== API =====
void microros_bridge_init()
{
    xTaskCreatePinnedToCore(microros_init_task, "mr_init", 16384, NULL, 5, NULL, 0);
}
