#include <stdbool.h>
#include <stdio.h>

#include "adafruit_mqtt.h"
#include "bmp180_sensor.h"
#include "bmp280_sensor.h"
#include "cloud_dashboard.h"
#include "config.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_bus.h"
#include "led_blink.h"
#include "wifi_connect.h"

static const char *TAG = "main";

#if SENSOR_TYPE != SENSOR_DASHBOARD
typedef enum {
    BOARD_BMP180 = 0,
    BOARD_BMP280,
} board_type_t;

static board_type_t s_board = BOARD_BMP180;

#define CHIP_ID_REG     0xD0
#define CHIP_ID_BMP180  0x55
#define CHIP_ID_BMP280  0x58
#define CHIP_ID_BME280  0x60

static esp_err_t probe_chip_id(uint8_t addr, uint8_t *chip_id)
{
    i2c_master_dev_handle_t dev = NULL;
    esp_err_t err = i2c_bus_add_device(addr, &dev);
    if (err != ESP_OK) {
        return err;
    }

    uint8_t reg = CHIP_ID_REG;
    err = i2c_bus_write_read(dev, &reg, 1, chip_id, 1);
    i2c_bus_remove_device(dev);
    return err;
}

static esp_err_t detect_board(board_type_t *out)
{
#if SENSOR_TYPE == SENSOR_BMP180
    *out = BOARD_BMP180;
    ESP_LOGI(TAG, "Board type forced: BMP180");
    return ESP_OK;
#elif SENSOR_TYPE == SENSOR_BMP280
    *out = BOARD_BMP280;
    ESP_LOGI(TAG, "Board type forced: BMP/BME280");
    return ESP_OK;
#else
    const uint8_t addrs[] = { 0x77, 0x76 };
    for (size_t i = 0; i < sizeof(addrs); i++) {
        uint8_t chip_id = 0;
        if (probe_chip_id(addrs[i], &chip_id) != ESP_OK) {
            continue;
        }
        ESP_LOGI(TAG, "Probe 0x%02X -> chip ID 0x%02X", addrs[i], chip_id);
        if (chip_id == CHIP_ID_BMP180) {
            *out = BOARD_BMP180;
            return ESP_OK;
        }
        if (chip_id == CHIP_ID_BMP280 || chip_id == CHIP_ID_BME280) {
            *out = BOARD_BMP280;
            return ESP_OK;
        }
    }
    ESP_LOGE(TAG, "No supported sensor found (BMP180 / BMP280 / BME280)");
    return ESP_ERR_NOT_FOUND;
#endif
}

static void feeds_for_board(board_type_t board, adafruit_feeds_t *feeds)
{
    if (board == BOARD_BMP180) {
        feeds->feed_temp = ADAFRUIT_FEED_BMP180_TEMP;
        feeds->feed_pressure = ADAFRUIT_FEED_BMP180_PRESSURE;
        feeds->feed_humidity = NULL;
    } else {
        feeds->feed_temp = ADAFRUIT_FEED_BMP280_TEMP;
        feeds->feed_pressure = ADAFRUIT_FEED_BMP280_PRESSURE;
        feeds->feed_humidity = ADAFRUIT_FEED_BMP280_HUMIDITY;
    }
}

static esp_err_t sensor_init_for_board(board_type_t board)
{
    if (board == BOARD_BMP180) {
        return bmp180_init();
    }
    return bmp280_init();
}

static esp_err_t sensor_read_for_board(board_type_t board,
                                       float *temperature_c,
                                       float *pressure_hpa,
                                       float *humidity_percent,
                                       bool *has_humidity)
{
    *has_humidity = false;
    *humidity_percent = 0.0f;

    if (board == BOARD_BMP180) {
        bmp180_data_t reading;
        esp_err_t err = bmp180_read(&reading);
        if (err != ESP_OK) {
            return err;
        }
        *temperature_c = reading.temperature_c;
        *pressure_hpa = reading.pressure_hpa;
        ESP_LOGI(TAG, "BMP180 | Temp: %.2f C | Pressure: %.2f hPa",
                 *temperature_c, *pressure_hpa);
        return ESP_OK;
    }

    bmp280_data_t reading;
    esp_err_t err = bmp280_read(&reading);
    if (err != ESP_OK) {
        return err;
    }
    *temperature_c = reading.temperature_c;
    *pressure_hpa = reading.pressure_hpa;
    *humidity_percent = reading.humidity_percent;
    *has_humidity = reading.has_humidity;
    if (*has_humidity) {
        ESP_LOGI(TAG, "BMP/BME280 | Temp: %.2f C | Pressure: %.2f hPa | Humidity: %.2f %%",
                 *temperature_c, *pressure_hpa, *humidity_percent);
    } else {
        ESP_LOGI(TAG, "BMP280 | Temp: %.2f C | Pressure: %.2f hPa",
                 *temperature_c, *pressure_hpa);
    }
    return ESP_OK;
}
#endif

static void wait_for_wifi(void)
{
    while (!wifi_is_connected()) {
        ESP_LOGW(TAG, "Waiting for WiFi...");
        led_indicate(LED_PATTERN_WIFI_WAIT);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

#if SENSOR_TYPE != SENSOR_DASHBOARD
static esp_err_t sample_once(void)
{
    float temperature_c = 0.0f;
    float pressure_hpa = 0.0f;
    float humidity_percent = 0.0f;
    bool has_humidity = false;

    if (!wifi_is_connected()) {
        led_indicate(LED_PATTERN_WIFI_WAIT);
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = sensor_read_for_board(s_board, &temperature_c, &pressure_hpa,
                                          &humidity_percent, &has_humidity);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Sensor read failed");
        led_indicate(LED_PATTERN_SENSOR_FAIL);
        return err;
    }

    err = adafruit_mqtt_publish_sensor(temperature_c, pressure_hpa,
                                       humidity_percent, has_humidity);
    if (err == ESP_OK) {
        led_indicate(LED_PATTERN_OK);
    } else {
        led_indicate(LED_PATTERN_PUBLISH_FAIL);
    }
    return err;
}

#if !ENABLE_DEEP_SLEEP && SENSOR_TYPE != SENSOR_DASHBOARD
/* TWDT timeout is typically 5s; feed it while waiting between samples. */
static void delay_ms_feed_wdt(uint32_t total_ms, bool wdt_subscribed)
{
    const uint32_t slice_ms = 1000;
    uint32_t remaining = total_ms;

    while (remaining > 0) {
        uint32_t step = (remaining > slice_ms) ? slice_ms : remaining;
        vTaskDelay(pdMS_TO_TICKS(step));
        if (wdt_subscribed) {
            esp_task_wdt_reset();
        }
        remaining -= step;
    }
}

static void sensor_task(void *arg)
{
    (void)arg;

    bool wdt_subscribed = (esp_task_wdt_add(NULL) == ESP_OK);
    if (!wdt_subscribed) {
        ESP_LOGW(TAG, "Task WDT not available for sensor_task");
    }

    while (1) {
        if (wdt_subscribed) {
            esp_task_wdt_reset();
        }
        sample_once();
        delay_ms_feed_wdt(SENSOR_READ_MS, wdt_subscribed);
    }
}
#endif
#endif

#if SENSOR_TYPE == SENSOR_DASHBOARD
static void run_dashboard(void)
{
    ESP_LOGI(TAG, "Dashboard firmware: Adafruit IO reader, no local sensor");

    if (led_init() != ESP_OK) {
        ESP_LOGW(TAG, "LED init failed, continuing without blink");
    }

    esp_err_t wifi_err = wifi_connect_init();
    if (wifi_err != ESP_OK && wifi_err != ESP_ERR_TIMEOUT) {
        ESP_LOGE(TAG, "WiFi init failed");
        return;
    }
    wait_for_wifi();

    if (cloud_dashboard_start() != ESP_OK) {
        ESP_LOGE(TAG, "Dashboard start failed");
        return;
    }

    while (cloud_dashboard_wait_connected(pdMS_TO_TICKS(1000)) != ESP_OK) {
        ESP_LOGW(TAG, "Waiting for MQTT...");
        led_indicate(LED_PATTERN_MQTT_WAIT);
        if (!wifi_is_connected()) {
            wait_for_wifi();
        }
    }

    char ip[16];
    if (wifi_get_ip_str(ip, sizeof(ip)) == ESP_OK) {
        ESP_LOGI(TAG, "Web page: http://%s/", ip);
    }
    led_indicate(LED_PATTERN_OK);
}
#endif

void app_main(void)
{
#if SENSOR_TYPE == SENSOR_DASHBOARD
    run_dashboard();
#else
    ESP_LOGI(TAG, "ESP32 sensor firmware starting");

    if (led_init() != ESP_OK) {
        ESP_LOGW(TAG, "LED init failed, continuing without blink");
    }

    esp_err_t wifi_err = wifi_connect_init();
    if (wifi_err != ESP_OK && wifi_err != ESP_ERR_TIMEOUT) {
        ESP_LOGE(TAG, "WiFi init failed");
        return;
    }
    wait_for_wifi();

    if (i2c_bus_init() != ESP_OK) {
        ESP_LOGE(TAG, "I2C init failed");
        return;
    }

    i2c_bus_scan();

    if (detect_board(&s_board) != ESP_OK) {
        led_indicate(LED_PATTERN_SENSOR_FAIL);
        return;
    }

    if (sensor_init_for_board(s_board) != ESP_OK) {
        ESP_LOGE(TAG, "Sensor init failed");
        led_indicate(LED_PATTERN_SENSOR_FAIL);
        return;
    }

    adafruit_feeds_t feeds;
    feeds_for_board(s_board, &feeds);

    if (adafruit_mqtt_start(&feeds) != ESP_OK) {
        ESP_LOGE(TAG, "Adafruit MQTT start failed");
        return;
    }

    while (adafruit_mqtt_wait_connected(pdMS_TO_TICKS(1000)) != ESP_OK) {
        ESP_LOGW(TAG, "Waiting for MQTT...");
        led_indicate(LED_PATTERN_MQTT_WAIT);
        if (!wifi_is_connected()) {
            wait_for_wifi();
        }
    }

#if ENABLE_DEEP_SLEEP
    sample_once();
    /* Allow QoS 1 PUBACKs / status publish to leave the radio */
    vTaskDelay(pdMS_TO_TICKS(2000));
    ESP_LOGI(TAG, "Deep sleep for %d ms", SENSOR_READ_MS);
    esp_deep_sleep(SENSOR_READ_MS * 1000ULL);
#else
    xTaskCreate(sensor_task, "sensor_task", 6144, NULL, 5, NULL);
#endif
#endif
}
