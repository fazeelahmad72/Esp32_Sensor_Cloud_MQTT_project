#pragma once

#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Display node: subscribe to the BMP180 and BMP/BME280 Adafruit IO feeds
 * and serve an interactive local dashboard. No I2C sensor is used.
 */
esp_err_t cloud_dashboard_start(void);
esp_err_t cloud_dashboard_wait_connected(TickType_t ticks_to_wait);

#ifdef __cplusplus
}
#endif
