#include "led_blink.h"

#include "config.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "led";
static bool s_ready = false;

esp_err_t led_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << LED_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    esp_err_t err = gpio_config(&io);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "LED GPIO config failed");
        return err;
    }

    gpio_set_level(LED_PIN, 0);
    s_ready = true;
    ESP_LOGI(TAG, "LED ready on GPIO%d", LED_PIN);
    return ESP_OK;
}

static void pulse(int on_ms, int off_ms)
{
    gpio_set_level(LED_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(on_ms));
    gpio_set_level(LED_PIN, 0);
    if (off_ms > 0) {
        vTaskDelay(pdMS_TO_TICKS(off_ms));
    }
}

void led_blink(void)
{
    led_indicate(LED_PATTERN_OK);
}

void led_indicate(led_pattern_t pattern)
{
    if (!s_ready) {
        return;
    }

    switch (pattern) {
    case LED_PATTERN_OK:
        pulse(LED_BLINK_MS, 0);
        break;
    case LED_PATTERN_WIFI_WAIT:
        pulse(400, 400);
        break;
    case LED_PATTERN_MQTT_WAIT:
        pulse(80, 80);
        pulse(80, 200);
        break;
    case LED_PATTERN_SENSOR_FAIL:
        pulse(80, 80);
        pulse(80, 80);
        pulse(80, 200);
        break;
    case LED_PATTERN_PUBLISH_FAIL:
        pulse(500, 0);
        break;
    default:
        break;
    }
}
