#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Opaque device handle: stores 7-bit I2C address */
typedef struct {
    uint8_t address;
} i2c_device_t;

typedef i2c_device_t *i2c_master_dev_handle_t;

esp_err_t i2c_bus_init(void);

/* Prints every device that answers on the I2C bus */
void i2c_bus_scan(void);

esp_err_t i2c_bus_add_device(uint8_t address, i2c_master_dev_handle_t *dev_handle);
esp_err_t i2c_bus_remove_device(i2c_master_dev_handle_t dev_handle);
esp_err_t i2c_bus_write(i2c_master_dev_handle_t dev, const uint8_t *data, size_t len);
esp_err_t i2c_bus_write_read(i2c_master_dev_handle_t dev,
                             const uint8_t *write_data, size_t write_len,
                             uint8_t *read_data, size_t read_len);

#ifdef __cplusplus
}
#endif
