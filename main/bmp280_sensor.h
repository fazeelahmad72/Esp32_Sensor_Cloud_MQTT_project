#pragma once

#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * I2C-only BMP280 / BME280 driver for 4-pin modules:
 *   Vin -> 3.3V, GND -> GND, SDA -> GPIO21, SCL -> GPIO22
 * No CS / SDO / SPI pins are used. Address is probed as 0x76 then 0x77.
 */
typedef struct {
    float temperature_c;
    float pressure_hpa;
    float humidity_percent; /* valid when has_humidity is true (BME280) */
    bool has_humidity;
} bmp280_data_t;

esp_err_t bmp280_init(void);
esp_err_t bmp280_read(bmp280_data_t *data);

#ifdef __cplusplus
}
#endif
