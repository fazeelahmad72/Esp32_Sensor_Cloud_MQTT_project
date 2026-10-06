#include "bmp180_sensor.h"

#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_bus.h"

static const char *TAG = "bmp180";

#define BMP180_ADDR          0x77
#define BMP180_REG_CHIP_ID   0xD0
#define BMP180_REG_CTRL      0xF4
#define BMP180_REG_DATA_MSB  0xF6
#define BMP180_REG_CALIB     0xAA
#define BMP180_CHIP_ID       0x55
#define BMP180_CMD_TEMP      0x2E
#define BMP180_CMD_PRESS     0x34
#define BMP180_OSS           1   /* standard oversampling; 0=ULP ... 3=UHR */

typedef struct {
    int16_t ac1;
    int16_t ac2;
    int16_t ac3;
    uint16_t ac4;
    uint16_t ac5;
    uint16_t ac6;
    int16_t b1;
    int16_t b2;
    int16_t mb;
    int16_t mc;
    int16_t md;
} bmp180_calib_t;

static i2c_master_dev_handle_t s_dev = NULL;
static bmp180_calib_t s_calib;
static bool s_ready = false;

static esp_err_t read_reg(uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_bus_write_read(s_dev, &reg, 1, buf, len);
}

static esp_err_t write_reg(uint8_t reg, uint8_t value)
{
    uint8_t data[2] = { reg, value };
    return i2c_bus_write(s_dev, data, sizeof(data));
}

/*
 * FreeRTOS tick is often 10 ms (CONFIG_FREERTOS_HZ=100).
 * pdMS_TO_TICKS(5) becomes 0, so conversion waits were skipped and UT/UP
 * were read before the ADC finished — that produced wildly high temperatures.
 * Use a busy-wait for short, datasheet-critical delays.
 */
static void delay_conversion_us(uint32_t us)
{
    esp_rom_delay_us(us);
}

static int16_t be_s16(const uint8_t *p)
{
    return (int16_t)(((uint16_t)p[0] << 8) | p[1]);
}

static uint16_t be_u16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

static bool calib_plausible(const bmp180_calib_t *c)
{
    /* Unprogrammed / failed EEPROM reads show as 0 or 0xFFFF */
    if (c->ac1 == 0 || c->ac1 == -1) {
        return false;
    }
    if (c->ac5 == 0 || c->ac5 == 0xFFFF) {
        return false;
    }
    if (c->ac6 == 0 || c->ac6 == 0xFFFF) {
        return false;
    }
    if (c->md == 0) {
        return false;
    }
    return true;
}

static esp_err_t load_calibration(void)
{
    uint8_t raw[22];
    esp_err_t err = read_reg(BMP180_REG_CALIB, raw, sizeof(raw));
    if (err != ESP_OK) {
        return err;
    }

    s_calib.ac1 = be_s16(&raw[0]);
    s_calib.ac2 = be_s16(&raw[2]);
    s_calib.ac3 = be_s16(&raw[4]);
    s_calib.ac4 = be_u16(&raw[6]);
    s_calib.ac5 = be_u16(&raw[8]);
    s_calib.ac6 = be_u16(&raw[10]);
    s_calib.b1 = be_s16(&raw[12]);
    s_calib.b2 = be_s16(&raw[14]);
    s_calib.mb = be_s16(&raw[16]);
    s_calib.mc = be_s16(&raw[18]);
    s_calib.md = be_s16(&raw[20]);

    ESP_LOGI(TAG,
             "Calib AC1=%d AC2=%d AC3=%d AC4=%u AC5=%u AC6=%u B1=%d B2=%d MB=%d MC=%d MD=%d",
             s_calib.ac1, s_calib.ac2, s_calib.ac3, s_calib.ac4, s_calib.ac5, s_calib.ac6,
             s_calib.b1, s_calib.b2, s_calib.mb, s_calib.mc, s_calib.md);

    if (!calib_plausible(&s_calib)) {
        ESP_LOGE(TAG, "Calibration coefficients look invalid");
        return ESP_ERR_INVALID_RESPONSE;
    }
    return ESP_OK;
}

static esp_err_t read_raw_temperature(int32_t *ut)
{
    esp_err_t err = write_reg(BMP180_REG_CTRL, BMP180_CMD_TEMP);
    if (err != ESP_OK) {
        return err;
    }

    /* Datasheet max conversion time is 4.5 ms */
    delay_conversion_us(5000);

    uint8_t data[2];
    err = read_reg(BMP180_REG_DATA_MSB, data, sizeof(data));
    if (err != ESP_OK) {
        return err;
    }

    /* UT is an unsigned 16-bit ADC value */
    *ut = (int32_t)be_u16(data);
    return ESP_OK;
}

static esp_err_t read_raw_pressure(int32_t *up)
{
    esp_err_t err = write_reg(BMP180_REG_CTRL, (uint8_t)(BMP180_CMD_PRESS + (BMP180_OSS << 6)));
    if (err != ESP_OK) {
        return err;
    }

    /* Datasheet max conversion times by OSS: 4.5 / 7.5 / 13.5 / 25.5 ms */
    static const uint32_t wait_us[] = { 5000, 8000, 14000, 26000 };
    delay_conversion_us(wait_us[BMP180_OSS]);

    uint8_t data[3];
    err = read_reg(BMP180_REG_DATA_MSB, data, sizeof(data));
    if (err != ESP_OK) {
        return err;
    }

    *up = (((int32_t)data[0] << 16) | ((int32_t)data[1] << 8) | (int32_t)data[2]) >> (8 - BMP180_OSS);
    return ESP_OK;
}

esp_err_t bmp180_init(void)
{
    esp_err_t err = i2c_bus_add_device(BMP180_ADDR, &s_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not add BMP180 on I2C");
        return err;
    }

    uint8_t chip_id = 0;
    err = read_reg(BMP180_REG_CHIP_ID, &chip_id, 1);
    if (err != ESP_OK || chip_id != BMP180_CHIP_ID) {
        ESP_LOGE(TAG, "Bad chip ID: 0x%02X (expected 0x%02X)", chip_id, BMP180_CHIP_ID);
        return ESP_ERR_NOT_FOUND;
    }

    err = load_calibration();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Calibration read failed");
        return err;
    }

    s_ready = true;
    ESP_LOGI(TAG, "BMP180 ready (addr 0x%02X, oss=%d)", BMP180_ADDR, BMP180_OSS);
    return ESP_OK;
}

esp_err_t bmp180_read(bmp180_data_t *data)
{
    if (!s_ready || data == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    int32_t ut = 0;
    int32_t up = 0;

    esp_err_t err = read_raw_temperature(&ut);
    if (err != ESP_OK) {
        return err;
    }

    err = read_raw_pressure(&up);
    if (err != ESP_OK) {
        return err;
    }

    /*
     * Bosch BMP180 datasheet compensation (integer form).
     * T is in 0.1 C, pressure p is in Pa.
     */
    int32_t x1 = ((ut - (int32_t)s_calib.ac6) * (int32_t)s_calib.ac5) >> 15;
    /* Multiply instead of left-shifting a (often negative) MC — avoids UB */
    int32_t x2 = ((int32_t)s_calib.mc * 2048) / (x1 + (int32_t)s_calib.md);
    int32_t b5 = x1 + x2;
    float temperature = (float)((b5 + 8) >> 4) / 10.0f;

    int32_t b6 = b5 - 4000;
    x1 = ((int32_t)s_calib.b2 * ((b6 * b6) >> 12)) >> 11;
    x2 = ((int32_t)s_calib.ac2 * b6) >> 11;
    int32_t x3 = x1 + x2;
    int32_t b3 = (((((int32_t)s_calib.ac1 * 4 + x3) << BMP180_OSS) + 2) / 4);

    x1 = ((int32_t)s_calib.ac3 * b6) >> 13;
    x2 = ((int32_t)s_calib.b1 * ((b6 * b6) >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    uint32_t b4 = ((uint32_t)s_calib.ac4 * (uint32_t)(x3 + 32768)) >> 15;
    uint32_t b7 = ((uint32_t)(up - b3)) * (50000UL >> BMP180_OSS);

    int32_t p;
    if (b7 < 0x80000000UL) {
        p = (int32_t)((b7 * 2) / b4);
    } else {
        p = (int32_t)((b7 / b4) * 2);
    }

    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    p = p + ((x1 + x2 + 3791) >> 4);

    data->temperature_c = temperature;
    data->pressure_hpa = (float)p / 100.0f;

    ESP_LOGD(TAG, "Raw UT=%ld UP=%ld -> T=%.2f C P=%.2f hPa",
             (long)ut, (long)up, temperature, data->pressure_hpa);
    return ESP_OK;
}
