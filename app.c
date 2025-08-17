#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// Конфиг/буферизация
#include "plotter_config.h"
#include "ring_buffer.h"

// HAL + SPI feeder + micro-ROS мост
#include "esp32_to_stm32.h"
#include "spi_feeder.h"
#include "microros_bridge.h"

static const char *TAG = "app";

static rb_t g_rb;
static uint8_t g_rb_mem[RB_SIZE_BYTES];

void appMain(void)
{
    ESP_LOGI(TAG, "boot");

    // 1) Кольцевой буфер
    rb_init(&g_rb, g_rb_mem, sizeof(g_rb_mem));
    ESP_LOGI(TAG, "Ring buffer initialized: %u bytes", (unsigned)RB_SIZE_BYTES);

    // 2) Инициализация железа (SPI/UART/READY)
    ESP_LOGI(TAG, "Initializing plotter hardware...");
    plotter_init();
    plotter_startfr_all_tasks();  // UART rx loop, control/heartbeat, keypad

    vTaskDelay(pdMS_TO_TICKS(500)); // stab

    // 3) Запуск SPI-фидера (читает из RB и шлёт в STM32 по 2048 B)
    ESP_LOGI(TAG, "Starting SPI feeder...");
    spi_feeder_start(&g_rb);

    // 4) Запуск micro-ROS моста (запрос данных, приём 480 B пакетов)
    ESP_LOGI(TAG, "Starting micro-ROS bridge...");
    microros_bridge_init(&g_rb);

    ESP_LOGI(TAG, "ready");

    // 5) Периодический статус
    uint32_t uptime_s = 0;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        uptime_s += 5;

        plotter_state_t ps;
        plotter_get_state(&ps);

        ESP_LOGI(TAG,
                 "alive: %" PRIu32 "s, RB=%zu/%u, Plotter: conn=%d cal=%d home=%d X=%" PRId32 " Y=%" PRId32,
                 uptime_s,
                 rb_used(&g_rb), (unsigned)RB_SIZE_BYTES,
                 ps.is_connected, ps.is_calibrated, ps.is_homed,
                 ps.x_pos, ps.y_pos);
    }
}
