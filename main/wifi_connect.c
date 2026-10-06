#include "wifi_connect.h"

#include <stdio.h>
#include <string.h>

#include "config.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/timers.h"
#include "nvs_flash.h"

static const char *TAG = "wifi";

#define WIFI_CONNECTED_BIT BIT0

#define WIFI_CONNECT_TIMEOUT_MS 30000
#define WIFI_RETRY_BASE_MS      1000
#define WIFI_RETRY_MAX_MS       30000

static EventGroupHandle_t s_wifi_events;
static TimerHandle_t s_reconnect_timer;
static esp_netif_t *s_sta_netif = NULL;
static int s_retry_count = 0;
static bool s_connected = false;

#if WIFI_USE_STATIC_IP
static esp_err_t apply_static_dns(esp_netif_t *netif)
{
    esp_netif_dns_info_t dns = { 0 };
    dns.ip.type = ESP_IPADDR_TYPE_V4;
    if (esp_netif_str_to_ip4(WIFI_STATIC_DNS, &dns.ip.u_addr.ip4) != ESP_OK) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = esp_netif_set_dns_info(netif, ESP_NETIF_DNS_MAIN, &dns);
    if (err != ESP_OK) {
        return err;
    }

    dns.ip.type = ESP_IPADDR_TYPE_V4;
    if (esp_netif_str_to_ip4(WIFI_STATIC_DNS_FALLBACK, &dns.ip.u_addr.ip4) != ESP_OK) {
        return ESP_ERR_INVALID_ARG;
    }
    return esp_netif_set_dns_info(netif, ESP_NETIF_DNS_BACKUP, &dns);
}
#endif

static uint32_t retry_delay_ms(int retry)
{
    uint32_t delay = WIFI_RETRY_BASE_MS;
    for (int i = 1; i < retry && delay < WIFI_RETRY_MAX_MS; i++) {
        delay *= 2;
    }
    return (delay > WIFI_RETRY_MAX_MS) ? WIFI_RETRY_MAX_MS : delay;
}

static void reconnect_timer_cb(TimerHandle_t timer)
{
    (void)timer;
    ESP_LOGI(TAG, "Retrying WiFi connect (attempt %d)", s_retry_count);
    esp_wifi_connect();
}

static void schedule_reconnect(void)
{
    s_retry_count++;
    uint32_t delay_ms = retry_delay_ms(s_retry_count);
    ESP_LOGW(TAG, "WiFi disconnected, reconnect in %lu ms", (unsigned long)delay_ms);
    xTimerStop(s_reconnect_timer, 0);
    xTimerChangePeriod(s_reconnect_timer, pdMS_TO_TICKS(delay_ms), 0);
    xTimerStart(s_reconnect_timer, 0);
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        s_connected = false;
        xEventGroupClearBits(s_wifi_events, WIFI_CONNECTED_BIT);
        schedule_reconnect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
#if WIFI_USE_STATIC_IP
        if (s_sta_netif != NULL && apply_static_dns(s_sta_netif) == ESP_OK) {
            ESP_LOGI(TAG, "DNS %s / %s", WIFI_STATIC_DNS, WIFI_STATIC_DNS_FALLBACK);
        }
#endif
        s_retry_count = 0;
        s_connected = true;
        xTimerStop(s_reconnect_timer, 0);
        xEventGroupSetBits(s_wifi_events, WIFI_CONNECTED_BIT);
    }
}

esp_err_t wifi_connect_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_flash_init failed");
        return err;
    }

    s_wifi_events = xEventGroupCreate();
    if (s_wifi_events == NULL) {
        return ESP_ERR_NO_MEM;
    }

    s_reconnect_timer = xTimerCreate("wifi_reconn", pdMS_TO_TICKS(WIFI_RETRY_BASE_MS),
                                     pdFALSE, NULL, reconnect_timer_cb);
    if (s_reconnect_timer == NULL) {
        return ESP_ERR_NO_MEM;
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    s_sta_netif = esp_netif_create_default_wifi_sta();
    if (s_sta_netif == NULL) {
        ESP_LOGE(TAG, "Failed to create WiFi STA netif");
        return ESP_FAIL;
    }

#if WIFI_USE_STATIC_IP
    esp_err_t ip_err = esp_netif_dhcpc_stop(s_sta_netif);
    if (ip_err != ESP_OK && ip_err != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STOPPED) {
        ESP_LOGE(TAG, "DHCP stop failed: %s", esp_err_to_name(ip_err));
        return ip_err;
    }

    esp_netif_ip_info_t ip_info = { 0 };
    if (esp_netif_str_to_ip4(WIFI_STATIC_IP, &ip_info.ip) != ESP_OK ||
        esp_netif_str_to_ip4(WIFI_STATIC_GATEWAY, &ip_info.gw) != ESP_OK ||
        esp_netif_str_to_ip4(WIFI_STATIC_NETMASK, &ip_info.netmask) != ESP_OK) {
        ESP_LOGE(TAG, "Invalid static IP settings");
        return ESP_ERR_INVALID_ARG;
    }
    ESP_ERROR_CHECK(esp_netif_set_ip_info(s_sta_netif, &ip_info));
    ESP_ERROR_CHECK(apply_static_dns(s_sta_netif));
    ESP_LOGI(TAG, "Static IP %s gw %s dns %s",
             WIFI_STATIC_IP, WIFI_STATIC_GATEWAY, WIFI_STATIC_DNS);
#endif

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = { 0 };
    strncpy((char *)wifi_config.sta.ssid, WIFI_SSID, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, WIFI_PASSWORD, sizeof(wifi_config.sta.password) - 1);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Connecting to WiFi SSID: %s", WIFI_SSID);

    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_events,
        WIFI_CONNECTED_BIT,
        pdFALSE,
        pdFALSE,
        pdMS_TO_TICKS(WIFI_CONNECT_TIMEOUT_MS));

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "WiFi connected");
        return ESP_OK;
    }

    ESP_LOGW(TAG, "WiFi not ready yet; background reconnect continues");
    return ESP_ERR_TIMEOUT;
}

bool wifi_is_connected(void)
{
    return s_connected;
}

esp_err_t wifi_get_ip_str(char *buf, size_t len)
{
    if (buf == NULL || len < 8 || !s_connected) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (netif == NULL) {
        return ESP_FAIL;
    }

    esp_netif_ip_info_t info;
    if (esp_netif_get_ip_info(netif, &info) != ESP_OK) {
        return ESP_FAIL;
    }

    snprintf(buf, len, IPSTR, IP2STR(&info.ip));
    return ESP_OK;
}
