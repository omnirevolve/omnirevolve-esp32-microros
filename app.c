#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// Конфиг/буферизация
#include "plotter_config.h"

// HAL + SPI feeder + micro-ROS мост
#include "esp32_to_stm32.h"
#include "microros_bridge.h"

static const char *TAG = "app";

void appMain(void)
{
    ESP_LOGI(TAG, "boot");

    plotter_init_sync();
    plotter_start_all_tasks();  // UART rx loop, control/heartbeat, keypad

    vTaskDelay(pdMS_TO_TICKS(500)); // stab

    ESP_LOGI(TAG, "Starting micro-ROS bridge...");
    microros_bridge_init();

    ESP_LOGI(TAG, "ready");

    uint32_t uptime_s = 0;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        uptime_s += 5;

        plotter_state_t ps;
        plotter_get_state(&ps);

        ESP_LOGI(TAG,
                 "alive: %" PRIu32 "s, Plotter: conn=%d cal=%d home=%d X=%" PRId32 " Y=%" PRId32,
                 uptime_s,
                 ps.is_connected, ps.is_calibrated, ps.is_homed,
                 ps.x_pos, ps.y_pos);
    }
}
