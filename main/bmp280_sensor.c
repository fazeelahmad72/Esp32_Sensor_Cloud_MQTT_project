#include "bmp280_sensor.h"

#include "config.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_bus.h"

static const char *TAG = "bme280";

#define REG_CHIP_ID    0xD0
#define REG_RESET      0xE0
#define REG_CTRL_HUM   0xF2
#define REG_STATUS     0xF3
#define REG_CTRL_MEAS  0xF4
#define REG_CONFIG     0xF5
#define REG_PRESS_MSB  0xF7
#define REG_CALIB_T_P  0x88
#define REG_CALIB_H1   0xA1
#define REG_CALIB_H2   0xE1
#define BMP280_CHIP_ID 0x58
#define BME280_CHIP_ID 0x60

typedef struct {
    uint16_t dig_t1;
    int16_t dig_t2;
    int16_t dig_t3;
    uint16_t dig_p1;
    int16_t dig_p2;
    int16_t dig_p3;
    int16_t dig_p4;
    int16_t dig_p5;
    int16_t dig_p6;
    int16_t dig_p7;
    int16_t dig_p8;
    int16_t dig_p9;
    uint8_t dig_h1;
    int16_t dig_h2;
    uint8_t dig_h3;
    int16_t dig_h4;
    int16_t dig_h5;
    int8_t dig_h6;
} bme280_calib_t;

static i2c_master_dev_handle_t s_dev = NULL;
static uint8_t s_addr = 0;
static uint8_t s_chip_id = 0;
static bool s_is_bme280 = false;
static bme280_calib_t s_calib;
static int32_t s_t_fine = 0;
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

static uint16_t le_u16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[1] << 8) | p[0]);
}

static int16_t le_s16(const uint8_t *p)
{
    return (int16_t)le_u16(p);
}

static esp_err_t load_calibration(void)
{
    uint8_t raw[24];
    esp_err_t err = read_reg(REG_CALIB_T_P, raw, sizeof(raw));
    if (err != ESP_OK) {
        return err;
    }

    /* BMP/BME280 calibration words are little-endian (datasheet) */
    s_calib.dig_t1 = le_u16(&raw[0]);
    s_calib.dig_t2 = le_s16(&raw[2]);
    s_calib.dig_t3 = le_s16(&raw[4]);
    s_calib.dig_p1 = le_u16(&raw[6]);
    s_calib.dig_p2 = le_s16(&raw[8]);
    s_calib.dig_p3 = le_s16(&raw[10]);
    s_calib.dig_p4 = le_s16(&raw[12]);
    s_calib.dig_p5 = le_s16(&raw[14]);
    s_calib.dig_p6 = le_s16(&raw[16]);
    s_calib.dig_p7 = le_s16(&raw[18]);
    s_calib.dig_p8 = le_s16(&raw[20]);
    s_calib.dig_p9 = le_s16(&raw[22]);

    if (s_calib.dig_t1 == 0 || s_calib.dig_t1 == 0xFFFF || s_calib.dig_p1 == 0) {
        ESP_LOGE(TAG, "Temperature/pressure calibration looks invalid");
        return ESP_ERR_INVALID_RESPONSE;
    }

    ESP_LOGI(TAG, "Calib T1=%u T2=%d T3=%d P1=%u",
             s_calib.dig_t1, s_calib.dig_t2, s_calib.dig_t3, s_calib.dig_p1);

    if (!s_is_bme280) {
        return ESP_OK;
    }

    err = read_reg(REG_CALIB_H1, &s_calib.dig_h1, 1);
    if (err != ESP_OK) {
        return err;
    }

    uint8_t h[7];
    err = read_reg(REG_CALIB_H2, h, sizeof(h));
    if (err != ESP_OK) {
        return err;
    }

    s_calib.dig_h2 = le_s16(&h[0]);
    s_calib.dig_h3 = h[2];
    /*
     * dig_H4 / dig_H5 are signed 12-bit values packed across three bytes:
     *   H4 = (E4 << 4) | (E5 & 0x0F)
     *   H5 = (E6 << 4) | (E5 >> 4)
     */
    s_calib.dig_h4 = (int16_t)(((int16_t)(int8_t)h[3] * 16) | (int16_t)(h[4] & 0x0F));
    s_calib.dig_h5 = (int16_t)(((int16_t)(int8_t)h[5] * 16) | (int16_t)(h[4] >> 4));
    s_calib.dig_h6 = (int8_t)h[6];

    ESP_LOGI(TAG, "Calib H1=%u H2=%d H3=%u H4=%d H5=%d H6=%d",
             s_calib.dig_h1, s_calib.dig_h2, s_calib.dig_h3,
             s_calib.dig_h4, s_calib.dig_h5, (int)s_calib.dig_h6);
    return ESP_OK;
}

/* Bosch integer compensation formulas from the BMP/BME280 datasheet */
static float compensate_temperature(int32_t adc_t)
{
    int32_t var1 = ((((adc_t >> 3) - ((int32_t)s_calib.dig_t1 << 1))) *
                    ((int32_t)s_calib.dig_t2)) >> 11;
    int32_t var2 = (((((adc_t >> 4) - ((int32_t)s_calib.dig_t1)) *
                      ((adc_t >> 4) - ((int32_t)s_calib.dig_t1))) >> 12) *
                    ((int32_t)s_calib.dig_t3)) >> 14;
    s_t_fine = var1 + var2;
    return (float)((s_t_fine * 5 + 128) >> 8) / 100.0f;
}

static float compensate_pressure(int32_t adc_p)
{
    int64_t var1 = ((int64_t)s_t_fine) - 128000;
    int64_t var2 = var1 * var1 * (int64_t)s_calib.dig_p6;
    var2 = var2 + ((var1 * (int64_t)s_calib.dig_p5) << 17);
    var2 = var2 + (((int64_t)s_calib.dig_p4) << 35);
    var1 = ((var1 * var1 * (int64_t)s_calib.dig_p3) >> 8) +
           ((var1 * (int64_t)s_calib.dig_p2) << 12);
    var1 = (((((int64_t)1) << 47) + var1) * ((int64_t)s_calib.dig_p1)) >> 33;

    if (var1 == 0) {
        return 0.0f;
    }

    int64_t p = 1048576 - adc_p;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)s_calib.dig_p9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)s_calib.dig_p8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)s_calib.dig_p7) << 4);

    /* p is in Q24.8 Pa; /25600 yields hPa */
    return (float)p / 25600.0f;
}

static float compensate_humidity(int32_t adc_h)
{
    int32_t v_x1 = s_t_fine - ((int32_t)76800);
    v_x1 = (((((adc_h << 14) - (((int32_t)s_calib.dig_h4) << 20) -
               (((int32_t)s_calib.dig_h5) * v_x1)) + ((int32_t)16384)) >> 15) *
            (((((((v_x1 * ((int32_t)s_calib.dig_h6)) >> 10) *
                 (((v_x1 * ((int32_t)s_calib.dig_h3)) >> 11) + ((int32_t)32768))) >> 10) +
               ((int32_t)2097152)) * ((int32_t)s_calib.dig_h2) + 8192) >> 14));
    v_x1 = (v_x1 - (((((v_x1 >> 15) * (v_x1 >> 15)) >> 7) * ((int32_t)s_calib.dig_h1)) >> 4));
    v_x1 = (v_x1 < 0) ? 0 : v_x1;
    v_x1 = (v_x1 > 419430400) ? 419430400 : v_x1;
    return (float)(v_x1 >> 12) / 1024.0f;
}

static esp_err_t wait_measurement_ready(void)
{
    for (int i = 0; i < 50; i++) {
        uint8_t status = 0;
        esp_err_t err = read_reg(REG_STATUS, &status, 1);
        if (err != ESP_OK) {
            return err;
        }
        /* Bit 3 = measuring; bit 0 = NVM copy in progress */
        if ((status & 0x09) == 0) {
            return ESP_OK;
        }
        esp_rom_delay_us(1000);
    }
    return ESP_ERR_TIMEOUT;
}

static esp_err_t try_address(uint8_t address)
{
    if (s_dev != NULL) {
        i2c_bus_remove_device(s_dev);
        s_dev = NULL;
    }

    esp_err_t err = i2c_bus_add_device(address, &s_dev);
    if (err != ESP_OK) {
        return err;
    }

    uint8_t chip_id = 0;
    err = read_reg(REG_CHIP_ID, &chip_id, 1);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "No response at 0x%02X", address);
        i2c_bus_remove_device(s_dev);
        s_dev = NULL;
        return err;
    }

    if (chip_id != BMP280_CHIP_ID && chip_id != BME280_CHIP_ID) {
        ESP_LOGW(TAG, "Addr 0x%02X chip ID 0x%02X (not BMP/BME280)", address, chip_id);
        i2c_bus_remove_device(s_dev);
        s_dev = NULL;
        return ESP_ERR_NOT_FOUND;
    }

    s_addr = address;
    s_chip_id = chip_id;
    s_is_bme280 = (chip_id == BME280_CHIP_ID);
    ESP_LOGI(TAG, "Found %s at 0x%02X (chip ID 0x%02X)",
             s_is_bme280 ? "BME280" : "BMP280", address, chip_id);
    return ESP_OK;
}

esp_err_t bmp280_init(void)
{
    /* 4-pin breakouts hardwire address via onboard SDO; try both common values */
    esp_err_t err = try_address(0x76);
    if (err != ESP_OK) {
        err = try_address(0x77);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BMP/BME280 not found on I2C 0x76 or 0x77");
        ESP_LOGE(TAG, "Wire: Vin=3.3V, GND=GND, SDA=GPIO%d, SCL=GPIO%d",
                 I2C_SDA_PIN, I2C_SCL_PIN);
        return ESP_ERR_NOT_FOUND;
    }

    err = write_reg(REG_RESET, 0xB6);
    if (err != ESP_OK) {
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    err = load_calibration();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Calibration read failed");
        return err;
    }

    /* Humidity oversampling must be set before ctrl_meas on BME280 */
    if (s_is_bme280) {
        err = write_reg(REG_CTRL_HUM, 0x01); /* humidity x1 */
        if (err != ESP_OK) {
            return err;
        }
    }

    /* standby 0.5 ms, filter off */
    err = write_reg(REG_CONFIG, 0x00);
    if (err != ESP_OK) {
        return err;
    }

    /*
     * Normal mode, temp x2, pressure x2 (0x4B = 010 010 11).
     * Slightly higher oversampling than x1 improves stability without
     * changing the datasheet compensation math.
     */
    err = write_reg(REG_CTRL_MEAS, 0x4B);
    if (err != ESP_OK) {
        return err;
    }

    /* Allow first conversion to complete before any read */
    esp_rom_delay_us(20000);
    (void)wait_measurement_ready();

    s_ready = true;
    ESP_LOGI(TAG, "%s ready (addr 0x%02X)", s_is_bme280 ? "BME280" : "BMP280", s_addr);
    return ESP_OK;
}

esp_err_t bmp280_read(bmp280_data_t *data)
{
    if (!s_ready || data == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = wait_measurement_ready();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Measurement ready wait failed (%s), reading anyway",
                 esp_err_to_name(err));
    }

    uint8_t raw[8] = { 0 };
    size_t read_len = s_is_bme280 ? 8 : 6;
    err = read_reg(REG_PRESS_MSB, raw, read_len);
    if (err != ESP_OK) {
        return err;
    }

    /* 20-bit ADC values: MSB << 12 | LSB << 4 | XLSB >> 4 */
    int32_t adc_p = (int32_t)(((uint32_t)raw[0] << 12) | ((uint32_t)raw[1] << 4) |
                              ((uint32_t)raw[2] >> 4));
    int32_t adc_t = (int32_t)(((uint32_t)raw[3] << 12) | ((uint32_t)raw[4] << 4) |
                              ((uint32_t)raw[5] >> 4));

    if (adc_t == 0x80000 || adc_p == 0x80000) {
        ESP_LOGW(TAG, "ADC returned disabled/skip value");
        return ESP_ERR_INVALID_RESPONSE;
    }

    data->temperature_c = compensate_temperature(adc_t);
    data->pressure_hpa = compensate_pressure(adc_p);
    data->has_humidity = s_is_bme280;
    data->humidity_percent = 0.0f;

    if (s_is_bme280) {
        int32_t adc_h = (int32_t)(((uint32_t)raw[6] << 8) | raw[7]);
        data->humidity_percent = compensate_humidity(adc_h);
    }

    ESP_LOGD(TAG, "Raw T=%ld P=%ld -> T=%.2f C P=%.2f hPa",
             (long)adc_t, (long)adc_p, data->temperature_c, data->pressure_hpa);
    return ESP_OK;
}
