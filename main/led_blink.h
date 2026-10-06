#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    LED_PATTERN_OK = 0,       /* single short blink — sample published */
    LED_PATTERN_WIFI_WAIT,    /* slow pulse while waiting for Wi-Fi */
    LED_PATTERN_MQTT_WAIT,    /* two short blinks — waiting for MQTT */
    LED_PATTERN_SENSOR_FAIL,  /* three short blinks */
    LED_PATTERN_PUBLISH_FAIL, /* long blink */
} led_pattern_t;

esp_err_t led_init(void);
void led_blink(void);
void led_indicate(led_pattern_t pattern);

#ifdef __cplusplus
}
#endif
