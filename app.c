#include <stdio.h>
#include <unistd.h>
#include <string.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/byte_multi_array.h>
#include <std_msgs/msg/float32.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#endif

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

// Buffer settings
#define BUFFER_SIZE 4096
#define BUFFER_THRESHOLD_PERCENT 25

static const char *TAG = "plotter";

// ROS entities
rcl_subscription_t subscriber;
rcl_publisher_t publisher;
std_msgs__msg__ByteMultiArray recv_msg;
std_msgs__msg__Float32 buffer_status_msg;

// Simple buffer management
static uint8_t plotter_buffer[BUFFER_SIZE];
static size_t buffer_write_pos = 0;
static size_t buffer_read_pos = 0;
static size_t buffer_count = 0;

// Timer for periodic buffer status publishing
rcl_timer_t timer;

// Weak implementations for ESP32 plotter functions
__attribute__((weak)) void esp32_plotter_init(void) {
    #ifdef ESP_PLATFORM
    ESP_LOGW(TAG, "Using stub esp32_plotter_init()");
    #else
    printf("Using stub esp32_plotter_init()\n");
    #endif
}

__attribute__((weak)) void esp32_plotter_send_data(uint8_t *data, size_t len) {
    printf("Stub: Would send %zu bytes to plotter\n", len);
    // Print first few bytes for debugging
    if (len > 0) {
        printf("Data: ");
        for (size_t i = 0; i < len && i < 8; i++) {
            printf("0x%02X ", data[i]);
        }
        printf("\n");
    }
}

__attribute__((weak)) bool esp32_plotter_is_ready(void) {
    return true; // Always ready in stub mode
}

// Calculate buffer free percentage
float get_buffer_free_percent(void) {
    return ((float)(BUFFER_SIZE - buffer_count) / (float)BUFFER_SIZE) * 100.0f;
}

// Buffer write function
bool buffer_write(const uint8_t *data, size_t len) {
    if (len > (BUFFER_SIZE - buffer_count)) {
        return false; // Not enough space
    }
    
    for (size_t i = 0; i < len; i++) {
        plotter_buffer[buffer_write_pos] = data[i];
        buffer_write_pos = (buffer_write_pos + 1) % BUFFER_SIZE;
        buffer_count++;
    }
    return true;
}

// Buffer read function
size_t buffer_read(uint8_t *data, size_t max_len) {
    size_t to_read = (buffer_count < max_len) ? buffer_count : max_len;
    
    for (size_t i = 0; i < to_read; i++) {
        data[i] = plotter_buffer[buffer_read_pos];
        buffer_read_pos = (buffer_read_pos + 1) % BUFFER_SIZE;
        buffer_count--;
    }
    return to_read;
}

// Subscription callback
void subscription_callback(const void *msgin) {
    const std_msgs__msg__ByteMultiArray *msg = (const std_msgs__msg__ByteMultiArray *)msgin;
    
    printf("Received %zu bytes from ROS2\n", msg->data.size);
    
    // Try to write to buffer
    if (buffer_write(msg->data.data, msg->data.size)) {
        printf("Data buffered. Free space: %.1f%%\n", get_buffer_free_percent());
        
        // Try to send to plotter if ready
        if (esp32_plotter_is_ready() && buffer_count > 0) {
            uint8_t temp[256];
            size_t to_send = buffer_read(temp, sizeof(temp));
            if (to_send > 0) {
                esp32_plotter_send_data(temp, to_send);
            }
        }
    } else {
        printf("Buffer overflow! Cannot store %zu bytes\n", msg->data.size);
    }
    
    // Publish buffer status if threshold crossed
    float free_percent = get_buffer_free_percent();
    if (free_percent > BUFFER_THRESHOLD_PERCENT) {
        buffer_status_msg.data = free_percent;
        RCSOFTCHECK(rcl_publish(&publisher, &buffer_status_msg, NULL));
        printf("Published buffer status: %.1f%% free\n", free_percent);
    }
}

// Timer callback for periodic status updates
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    (void)last_call_time;
    if (timer != NULL) {
        // Process any pending data in buffer
        if (esp32_plotter_is_ready() && buffer_count > 0) {
            uint8_t temp[256];
            size_t to_send = buffer_read(temp, sizeof(temp));
            if (to_send > 0) {
                esp32_plotter_send_data(temp, to_send);
                printf("Timer: Sent %zu bytes to plotter\n", to_send);
            }
        }
        
        // Publish current buffer status
        buffer_status_msg.data = get_buffer_free_percent();
        RCSOFTCHECK(rcl_publish(&publisher, &buffer_status_msg, NULL));
    }
}

void appMain(void * arg) {
    (void)arg;
    
    printf("Starting Plotter micro-ROS node...\n");
    
    // Initialize plotter hardware
    esp32_plotter_init();
    
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    
    #ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    printf("Waiting for WiFi...\n");
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    
    // Create init_options with UDP transport
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    
    rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
    
    // Wait for agent
    printf("Waiting for agent...\n");
    while (rmw_uros_ping_agent(1000, 1) != RMW_RET_OK) {
        printf("Agent not available, retrying...\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    printf("Agent connected!\n");
    
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
    #else
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    #endif
    
    // Create node
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "plotter_esp32_node", "", &support));
    printf("Node created\n");
    
    // Create subscriber
    RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, ByteMultiArray),
        "/plotter/data"));
    printf("Subscriber created\n");
    
    // Create publisher
    RCCHECK(rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "/plotter/buffer_status"));
    printf("Publisher created\n");
    
    // Create timer for periodic tasks (100ms)
    const unsigned int timer_timeout = 100;
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_callback));
    printf("Timer created\n");
    
    // Create executor
    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    
    // Initialize message memory
    recv_msg.data.capacity = 256;
    recv_msg.data.size = 0;
    recv_msg.data.data = (uint8_t*) malloc(recv_msg.data.capacity * sizeof(uint8_t));
    
    // Add subscription and timer to executor
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &recv_msg,
                                          &subscription_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    printf("Executor configured\n");
    
    // Publish initial buffer status
    buffer_status_msg.data = 100.0f;
    RCCHECK(rcl_publish(&publisher, &buffer_status_msg, NULL));
    printf("Initial buffer status published\n");
    
    printf("Entering main loop...\n");
    
    // Main loop - same as int32_publisher
    while(1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        usleep(100000);
    }
    
    // Cleanup (never reached)
    RCCHECK(rcl_subscription_fini(&subscriber, &node));
    RCCHECK(rcl_publisher_fini(&publisher, &node));
    RCCHECK(rcl_node_fini(&node));
    
    vTaskDelete(NULL);
}