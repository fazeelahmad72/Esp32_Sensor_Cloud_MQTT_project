#include "i2c_bus.h"

#include <stdlib.h>

#include "config.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "i2c_bus";

/*
 * Software I2C (bit-bang) on the pins in config.h (default D21/D22).
 * We drive SDA/SCL ourselves as open-drain GPIOs.
 */
#define SDA_PIN   ((gpio_num_t)I2C_SDA_PIN)
#define SCL_PIN   ((gpio_num_t)I2C_SCL_PIN)

/* Half-period delay for ~100 kHz */
#define I2C_DELAY_US  5

static bool s_ready = false;
static SemaphoreHandle_t s_lock = NULL;

static inline void sda_high(void)
{
    gpio_set_level(SDA_PIN, 1);
}

static inline void sda_low(void)
{
    gpio_set_level(SDA_PIN, 0);
}

static inline void scl_high(void)
{
    gpio_set_level(SCL_PIN, 1);
}

static inline void scl_low(void)
{
    gpio_set_level(SCL_PIN, 0);
}

static inline int sda_read(void)
{
    return gpio_get_level(SDA_PIN);
}

static inline int scl_read(void)
{
    return gpio_get_level(SCL_PIN);
}

static inline void i2c_delay(void)
{
    esp_rom_delay_us(I2C_DELAY_US);
}

static bool wait_scl_high(void)
{
    scl_high();
    for (int i = 0; i < 50; i++) {
        if (scl_read()) {
            return true;
        }
        esp_rom_delay_us(10);
    }
    return false;
}

static void i2c_start(void)
{
    sda_high();
    scl_high();
    i2c_delay();
    sda_low();
    i2c_delay();
    scl_low();
    i2c_delay();
}

static void i2c_stop(void)
{
    sda_low();
    i2c_delay();
    wait_scl_high();
    i2c_delay();
    sda_high();
    i2c_delay();
}

static bool i2c_write_byte(uint8_t byte)
{
    for (int i = 7; i >= 0; i--) {
        if (byte & (1 << i)) {
            sda_high();
        } else {
            sda_low();
        }
        i2c_delay();
        if (!wait_scl_high()) {
            return false;
        }
        i2c_delay();
        scl_low();
        i2c_delay();
    }

    /* Release SDA for ACK from slave */
    sda_high();
    i2c_delay();
    if (!wait_scl_high()) {
        return false;
    }
    i2c_delay();
    bool acked = (sda_read() == 0);
    scl_low();
    i2c_delay();
    return acked;
}

static uint8_t i2c_read_byte(bool ack)
{
    uint8_t byte = 0;
    sda_high(); /* release for slave to drive */

    for (int i = 7; i >= 0; i--) {
        if (!wait_scl_high()) {
            return 0xFF;
        }
        i2c_delay();
        if (sda_read()) {
            byte |= (1 << i);
        }
        scl_low();
        i2c_delay();
    }

    if (ack) {
        sda_low();
    } else {
        sda_high();
    }
    i2c_delay();
    wait_scl_high();
    i2c_delay();
    scl_low();
    sda_high();
    i2c_delay();
    return byte;
}

static bool bus_lines_ok(void)
{
    sda_high();
    scl_high();
    i2c_delay();
    i2c_delay();

    int sda = sda_read();
    int scl = scl_read();
    ESP_LOGI(TAG, "Bus idle check: SDA=%d SCL=%d (both should be 1)", sda, scl);

    if (sda == 0 || scl == 0) {
        ESP_LOGE(TAG, "I2C line stuck LOW (SDA=%d SCL=%d)", sda, scl);
        ESP_LOGE(TAG, "This is a WIRING problem, not a software bug.");
        ESP_LOGE(TAG, "Do this:");
        ESP_LOGE(TAG, "  1) Unplug BMP280 SDA wire from ESP32");
        ESP_LOGE(TAG, "  2) Reset ESP32 - SDA should become 1");
        ESP_LOGE(TAG, "  3) If SDA stays 0 with wire removed, GPIO%d on the board is bad/busy", I2C_SDA_PIN);
        ESP_LOGE(TAG, "  4) If SDA becomes 1, your sensor wire/module is shorting SDA to GND");
        ESP_LOGE(TAG, "Correct wiring: Vin=3.3V, GND=GND, SDA=GPIO%d, SCL=GPIO%d",
                 I2C_SDA_PIN, I2C_SCL_PIN);
        return false;
    }
    return true;
}

esp_err_t i2c_bus_init(void)
{
    if (s_ready) {
        return ESP_OK;
    }

    if (s_lock == NULL) {
        s_lock = xSemaphoreCreateMutex();
        if (s_lock == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }

    gpio_config_t io = {
        .pin_bit_mask = (1ULL << SDA_PIN) | (1ULL << SCL_PIN),
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    esp_err_t err = gpio_config(&io);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "GPIO config failed: %s", esp_err_to_name(err));
        return err;
    }

    sda_high();
    scl_high();
    i2c_delay();

    s_ready = true;
    ESP_LOGI(TAG, "Soft I2C ready on SDA=%d SCL=%d", I2C_SDA_PIN, I2C_SCL_PIN);
    return ESP_OK;
}

void i2c_bus_scan(void)
{
    if (!s_ready) {
        ESP_LOGE(TAG, "I2C bus not ready");
        return;
    }

    if (!bus_lines_ok()) {
        ESP_LOGE(TAG, "Skipping scan because bus is stuck");
        return;
    }

    ESP_LOGI(TAG, "Scanning I2C bus...");
    int found = 0;

    xSemaphoreTake(s_lock, portMAX_DELAY);
    for (uint8_t addr = 1; addr < 127; addr++) {
        i2c_start();
        bool ok = i2c_write_byte((uint8_t)(addr << 1)); /* write mode */
        i2c_stop();
        if (ok) {
            ESP_LOGI(TAG, "Found device at 0x%02X", addr);
            found++;
        }
    }
    xSemaphoreGive(s_lock);

    if (found == 0) {
        ESP_LOGW(TAG, "No I2C devices found. Check wiring:");
        ESP_LOGW(TAG, "  Vin -> 3.3V");
        ESP_LOGW(TAG, "  GND -> GND");
        ESP_LOGW(TAG, "  SDA -> GPIO%d", I2C_SDA_PIN);
        ESP_LOGW(TAG, "  SCL -> GPIO%d", I2C_SCL_PIN);
    } else if (found > 4) {
        ESP_LOGE(TAG, "Found %d devices - that is fake (SDA likely stuck LOW)", found);
    } else {
        ESP_LOGI(TAG, "Scan done. Found %d device(s)", found);
    }
}

esp_err_t i2c_bus_add_device(uint8_t address, i2c_master_dev_handle_t *dev_handle)
{
    if (!s_ready || dev_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    i2c_device_t *dev = calloc(1, sizeof(i2c_device_t));
    if (dev == NULL) {
        return ESP_ERR_NO_MEM;
    }
    dev->address = address;
    *dev_handle = dev;
    return ESP_OK;
}

esp_err_t i2c_bus_remove_device(i2c_master_dev_handle_t dev_handle)
{
    if (dev_handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    free(dev_handle);
    return ESP_OK;
}

esp_err_t i2c_bus_write(i2c_master_dev_handle_t dev, const uint8_t *data, size_t len)
{
    if (!s_ready || dev == NULL || data == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);
    i2c_start();
    if (!i2c_write_byte((uint8_t)(dev->address << 1))) {
        i2c_stop();
        xSemaphoreGive(s_lock);
        return ESP_FAIL;
    }

    for (size_t i = 0; i < len; i++) {
        if (!i2c_write_byte(data[i])) {
            i2c_stop();
            xSemaphoreGive(s_lock);
            return ESP_FAIL;
        }
    }

    i2c_stop();
    xSemaphoreGive(s_lock);
    return ESP_OK;
}

esp_err_t i2c_bus_write_read(i2c_master_dev_handle_t dev,
                             const uint8_t *write_data, size_t write_len,
                             uint8_t *read_data, size_t read_len)
{
    if (!s_ready || dev == NULL || write_data == NULL || read_data == NULL ||
        write_len == 0 || read_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);

    i2c_start();
    if (!i2c_write_byte((uint8_t)(dev->address << 1))) {
        i2c_stop();
        xSemaphoreGive(s_lock);
        return ESP_FAIL;
    }
    for (size_t i = 0; i < write_len; i++) {
        if (!i2c_write_byte(write_data[i])) {
            i2c_stop();
            xSemaphoreGive(s_lock);
            return ESP_FAIL;
        }
    }

    /* Repeated start for read */
    i2c_start();
    if (!i2c_write_byte((uint8_t)((dev->address << 1) | 0x01))) {
        i2c_stop();
        xSemaphoreGive(s_lock);
        return ESP_FAIL;
    }

    for (size_t i = 0; i < read_len; i++) {
        bool ack = (i + 1) < read_len;
        read_data[i] = i2c_read_byte(ack);
    }

    i2c_stop();
    xSemaphoreGive(s_lock);
    return ESP_OK;
}
