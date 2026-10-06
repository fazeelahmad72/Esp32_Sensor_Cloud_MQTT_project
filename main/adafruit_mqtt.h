#pragma once

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    const char *feed_temp;
    const char *feed_pressure;
    const char *feed_humidity; /* NULL or "" if unused */
} adafruit_feeds_t;

esp_err_t adafruit_mqtt_start(const adafruit_feeds_t *feeds);
esp_err_t adafruit_mqtt_wait_connected(TickType_t ticks_to_wait);
bool adafruit_mqtt_is_connected(void);

/*
 * Publish sensor values (QoS 1). If offline, stores the latest sample and
 * flushes it automatically on the next MQTT reconnect.
 */
esp_err_t adafruit_mqtt_publish_sensor(float temperature_c,
                                       float pressure_hpa,
                                       float humidity_percent,
                                       bool has_humidity);

#ifdef __cplusplus
}
#endif
