#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float temperature_c;
    float pressure_hpa;
} bmp180_data_t;

esp_err_t bmp180_init(void);
esp_err_t bmp180_read(bmp180_data_t *data);

#ifdef __cplusplus
}
#endif
