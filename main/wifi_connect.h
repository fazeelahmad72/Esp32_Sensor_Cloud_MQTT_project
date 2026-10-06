#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Starts Wi-Fi STA. Returns ESP_OK when an IP is obtained within the timeout.
 * On timeout returns ESP_ERR_TIMEOUT but keeps reconnecting in the background
 * with exponential backoff.
 */
esp_err_t wifi_connect_init(void);
bool wifi_is_connected(void);

/* Writes the station IPv4 address, e.g. "192.168.1.40". */
esp_err_t wifi_get_ip_str(char *buf, size_t len);

#ifdef __cplusplus
}
#endif
