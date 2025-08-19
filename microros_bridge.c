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
#include "freertos/semphr.h"

#include "esp32_to_stm32.h"

// ===================== DEFINES =======================
#define PACKETS_NUMBER_FOR_FIRST_REQUEST 1
#define RX_PACKET_BYTES 2048u

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

static uint8_t data_buf[2][SPI_CHUNK_SIZE];
static uint8_t data_tail[RX_PACKET_BYTES];
static volatile uint8_t data_buf_full[2] = {0, 0};
static volatile uint8_t data_requested_for_buf[2] = {0, 0};
static TaskHandle_t sender_task_handle = NULL;

// microros_bridge.c (где-нибудь рядом с глобальными переменными)
static SemaphoreHandle_t s_out_mutex = NULL;

static void init_mutex(void)
{
#if (configUSE_MUTEXES == 1)
    s_out_mutex = xSemaphoreCreateMutex();            // динамический мьютекс
    configASSERT(s_out_mutex != NULL);
#else
    s_out_mutex = xSemaphoreCreateBinary();
    configASSERT(s_out_mutex != NULL);
    xSemaphoreGive(s_out_mutex);                      // сделать «отпущенным»
#warning "configUSE_MUTEXES==0: используем бинарный семафор вместо мьютекса"
#endif
}

#define LOCK_DATA_REQUEST_FLAG_CHANGING()    xSemaphoreTake(s_out_mutex, portMAX_DELAY)
#define UNLOCK_DATA_REQUEST_FLAG_CHANGING()  xSemaphoreGive(s_out_mutex)

static uint8_t pending_buf_index = 0xFF;
// ===================== NEED PACKETS =======================

static void publish_need_packets(uint8_t n){
    s_msg_need.data = n;
    rcl_ret_t rc = rcl_publish(&pub_need, &s_msg_need, NULL);
    if (rc == RCL_RET_OK){
        ESP_LOGI(TAG, "SENDING: need_packets: %u", n);
    } else {
        ESP_LOGW(TAG, "need_packets publish failed rc=%d", (int)rc);
    }
}

static uint32_t stored_data_len = 0;
static uint32_t out_of_current_buf_len = 0;
static uint8_t next_filling_buf_index = 1;
static uint32_t tail_data_len = 0;

static void reset_sub_stream_cb_private_variables() {
    stored_data_len = 0;
    out_of_current_buf_len = 0;
    next_filling_buf_index = 0;
    tail_data_len = 0;
}

static void reset_plotter_streaming_variables() {
    data_buf_full[0] = 0;
    data_buf_full[1] = 0;
}
// ===================== PACKETS RECEIVING CALLBACK =======================
static void sub_stream_cb(const void * msgin)
{
    //ESP_LOGI(TAG, ">> sub_stream_cb entered");
    const std_msgs__msg__UInt8MultiArray *m = (const std_msgs__msg__UInt8MultiArray*)msgin;
    const uint8_t *incoming_data = m->data.data;
    size_t incoming_data_len = m->data.size;
    static const char PACKETS_RECEIVER_TAG[] = "ROS2 packets receiver";
    if (incoming_data_len == 0){
        ESP_LOGW(PACKETS_RECEIVER_TAG, ">> Recieved len == 0");
        return;
    }
    
    if (incoming_data_len != RX_PACKET_BYTES){
        ESP_LOGW(PACKETS_RECEIVER_TAG, ">> Unexpected payload len=%u (expected %u)",
                 (unsigned)incoming_data_len, (unsigned)RX_PACKET_BYTES);
    }

    if (!s_draw_active) {
        ESP_LOGW(PACKETS_RECEIVER_TAG, "Received data from plotter data publisher when draw_active == FALSE");
        return;
    }

    if (data_buf_full[next_filling_buf_index]) { // this receiver should be called only if we have empty buffer; otherwise, this is wrong pipeline:
        ESP_LOGE(PACKETS_RECEIVER_TAG, "Wrong data pipeline, can't continue: receiving terminated, data dropped. Fullfill state: %u:%u", data_buf_full[0], data_buf_full[1]);
        return;
    }

    // check if we have tail in the tail buf:
    if (tail_data_len != 0) { // copy tail to currently filling buf:
       memcpy(data_buf[next_filling_buf_index], data_tail, tail_data_len);
       tail_data_len = 0;
    }

    // check if we need write all the data in current buf:
    if ((stored_data_len + incoming_data_len) >= SPI_CHUNK_SIZE) {
        out_of_current_buf_len = stored_data_len + incoming_data_len - SPI_CHUNK_SIZE;
        memcpy(data_buf[next_filling_buf_index] + stored_data_len, incoming_data, incoming_data_len - out_of_current_buf_len);
        data_requested_for_buf[next_filling_buf_index] = 0; // it should be safe because we can't sending and filling the buf simultaneously
        data_buf_full[next_filling_buf_index] = 1;
        // identify if we have free transfer buffer or we have to store in tail buf:
        next_filling_buf_index = !next_filling_buf_index; // switch active buffer (should be normal flow)
        uint8_t store_in_tail_buf = data_buf_full[next_filling_buf_index]; // if the next normal buf is still full, we have to store data tail in tail buf
        uint8_t *target_buf_for_data_tail = store_in_tail_buf ? data_tail : data_buf[next_filling_buf_index];
        memcpy(target_buf_for_data_tail, incoming_data + incoming_data_len - out_of_current_buf_len, out_of_current_buf_len);

        if (store_in_tail_buf) {
            tail_data_len = out_of_current_buf_len;
        } else {
            stored_data_len = out_of_current_buf_len;
        }
        
        if (!store_in_tail_buf) {
            uint8_t send_request_for_more_data = 0;
            LOCK_DATA_REQUEST_FLAG_CHANGING();
            if (!data_requested_for_buf[next_filling_buf_index]) {
                send_request_for_more_data = 1;
                data_requested_for_buf[next_filling_buf_index] = 1;
            }
            UNLOCK_DATA_REQUEST_FLAG_CHANGING();
            if (send_request_for_more_data) {
                ESP_LOGI(PACKETS_RECEIVER_TAG, "Sent new packets request from PACKETS RECEIVER");
                publish_need_packets((SPI_CHUNK_SIZE - out_of_current_buf_len)/RX_PACKET_BYTES + (out_of_current_buf_len ? 1 : 0));
            }
        }
    }
    else {
        memcpy(data_buf[next_filling_buf_index] + stored_data_len, incoming_data, incoming_data_len);
        stored_data_len += incoming_data_len;
    }
    
    if (s_draw_active && s_finish_requested) {
        if (stored_data_len != SPI_CHUNK_SIZE) {
            memset(data_buf[next_filling_buf_index] + stored_data_len, 0, SPI_CHUNK_SIZE - stored_data_len); // fill with 0 all the data to make the packet eq SPI_CHUNK_SIZE
        }
        data_buf_full[next_filling_buf_index] = 1;
        reset_sub_stream_cb_private_variables();
    }
}

// ===================== DATA SENDER TO PLOTTER =======================
static void wait_on_task(uint32_t *delay_periods_counter, const char *tag, const char *info_message) {
    static const uint8_t one_period_wait_delay = 5;
    static const uint8_t log_after_s = 5;
    vTaskDelay(pdMS_TO_TICKS(one_period_wait_delay));
    if (++(*delay_periods_counter) > log_after_s * 1000 / one_period_wait_delay) {
        //ESP_LOGW(tag, "%s for more than %ds", info_message, log_after_s);
        *delay_periods_counter = 0;
    }                
}

static void send_data_to_plotter_task(void *arg)
{
    static uint8_t pipeline_is_broken = 0;
    static const char SENDER_TAG[] = "Plotter packets sender";

    if (pipeline_is_broken) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        ESP_LOGE(SENDER_TAG, "Wrong data pipeline, can't continue: sending terminated, data dropped");
    }

    ESP_LOGI(SENDER_TAG, ">> send_data_to_plotter_task started");
    (void)arg;

    //ulTaskNotifyTake(pdTRUE, 0);
    for(;;){
        if (!s_draw_active) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        uint32_t delay_periods_counter = 0;
        // TODO: make this loop stoppable:
        while (!plotter_is_ready_to_receive_draw_stream_data()) { // just wait when plotter ready
            wait_on_task(&delay_periods_counter, SENDER_TAG, "Waiting for plotter ready");
        }
       
        ESP_LOGI(SENDER_TAG, "Plotter ready");

        static uint8_t next_buffer_index = 0;

        delay_periods_counter = 0;
        // TODO: make this loop stoppable with timeout:
        while (!data_buf_full[next_buffer_index]) {
            wait_on_task(&delay_periods_counter, SENDER_TAG, "Waiting for fullfilled buffer ");
            if (s_finish_requested) {
                reset_plotter_streaming_variables();
                if (data_buf_full[0] || data_buf_full[1]) { // TODO: check logic when both the buffers are not empty
                    next_buffer_index = data_buf_full[0] ? 0 : 1;
                }
            }
        }
        
        ESP_LOGI(SENDER_TAG, "Plotter ready & have fullfilled buffer index %u, sending data...", next_buffer_index);
        plotter_send_draw_stream_data(data_buf[next_buffer_index], SPI_CHUNK_SIZE);
        data_buf_full[next_buffer_index] = 0;
        if (!s_finish_requested) {
            next_buffer_index = !next_buffer_index;
            uint8_t send_request_for_more_data = 0;
            LOCK_DATA_REQUEST_FLAG_CHANGING();
            if (!data_requested_for_buf[next_buffer_index]) {
                send_request_for_more_data = 1;
                data_requested_for_buf[next_buffer_index] = 1;
            }
            UNLOCK_DATA_REQUEST_FLAG_CHANGING();
            if (send_request_for_more_data){
                ESP_LOGI(SENDER_TAG, "Sent new packets request from DATA SENDER");
                publish_need_packets((SPI_CHUNK_SIZE / RX_PACKET_BYTES) + ((SPI_CHUNK_SIZE % RX_PACKET_BYTES) ? 1 : 0)); // have to send +1 if need
            }
            if (data_buf_full[next_buffer_index]) { // start working with the next buffer immediately without waiting (but there is no difference when plotter has 10kHz nibbles painting freq (2048bytes = 4096nibbles / 10000 ~ 41ms per 2048bytes))
                continue;
            }
        }
        if (s_finish_requested) {
            next_buffer_index = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }   
}

// ===================== DRAW_START / DRAW_FINISH MESSAGE HANDLERS =======================

static void sub_draw_start_cb(const void * msgin)
{
    ESP_LOGI(TAG, ">> Draw start received");
    (void)msgin;
    if (s_draw_active)
    {
        ESP_LOGW(TAG, "DRAW_START when previous session isn't finished; ignoring");
        return;
    }

    plotter_send_cmd(CMD_DRAW_BEGIN, NULL);
    s_draw_active = true;
    s_finish_requested = false;

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
    init_mutex();

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

    // --- ИСПРАВЛЕНИЕ: rclc_subscription_init требует 5 аргументов. Задаём явный QoS. ---
    rmw_qos_profile_t qos_reliable_kl1 = rmw_qos_profile_default;
    qos_reliable_kl1.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    qos_reliable_kl1.durability  = RMW_QOS_POLICY_DURABILITY_VOLATILE;
    qos_reliable_kl1.history     = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    qos_reliable_kl1.depth       = 1;
    // ------------------------------------------------------------------------------------

    // Subscriber: stream RELIABLE
    RCCHECK(rclc_subscription_init(
        &sub_stream, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8MultiArray),
        "/plotter/byte_stream",
        &qos_reliable_kl1));

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

    xTaskCreatePinnedToCore(send_data_to_plotter_task, "mr_sender", 6144, NULL, 5, &sender_task_handle, 1);
    xTaskCreatePinnedToCore(executor_task,    "mr_executor",  9216, NULL, 6, NULL, 0);

    vTaskDelete(NULL);
}

// ===== API =====
void microros_bridge_init()
{
    xTaskCreatePinnedToCore(microros_init_task, "mr_init", 16384, NULL, 5, NULL, 0);
}
