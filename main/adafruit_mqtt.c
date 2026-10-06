#include "adafruit_mqtt.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "config.h"
#include "esp_crt_bundle.h"
#include "esp_log.h"
#include "freertos/event_groups.h"
#include "mqtt_client.h"
#include "wifi_connect.h"

static const char *TAG = "adafruit_mqtt";

#define MQTT_CONNECTED_BIT BIT0

static esp_mqtt_client_handle_t s_client = NULL;
static EventGroupHandle_t s_mqtt_events;
static bool s_connected = false;
static char s_topic_temp[128];
static char s_topic_pressure[128];
static char s_topic_humidity[128];
static char s_topic_status[128];
static bool s_has_humidity_feed = false;

typedef struct {
    float temperature_c;
    float pressure_hpa;
    float humidity_percent;
    bool has_humidity;
    bool valid;
} pending_sample_t;

static pending_sample_t s_pending = { 0 };

static void build_topics(const adafruit_feeds_t *feeds)
{
    snprintf(s_topic_temp, sizeof(s_topic_temp), "%s/feeds/%s",
             ADAFRUIT_USERNAME, feeds->feed_temp);
    snprintf(s_topic_pressure, sizeof(s_topic_pressure), "%s/feeds/%s",
             ADAFRUIT_USERNAME, feeds->feed_pressure);
    snprintf(s_topic_status, sizeof(s_topic_status), "%s/feeds/%s",
             ADAFRUIT_USERNAME, ADAFRUIT_FEED_STATUS);

    if (feeds->feed_humidity != NULL && feeds->feed_humidity[0] != '\0') {
        snprintf(s_topic_humidity, sizeof(s_topic_humidity), "%s/feeds/%s",
                 ADAFRUIT_USERNAME, feeds->feed_humidity);
        s_has_humidity_feed = true;
    } else {
        s_topic_humidity[0] = '\0';
        s_has_humidity_feed = false;
    }
}

static esp_err_t publish_values(float temperature_c,
                                float pressure_hpa,
                                float humidity_percent,
                                bool has_humidity)
{
    char temp_str[32];
    char pressure_str[32];
    char humidity_str[32];
    snprintf(temp_str, sizeof(temp_str), "%.2f", temperature_c);
    snprintf(pressure_str, sizeof(pressure_str), "%.2f", pressure_hpa);

    int msg_temp = esp_mqtt_client_publish(s_client, s_topic_temp, temp_str,
                                           0, ADAFRUIT_MQTT_QOS, 0);
    int msg_pressure = esp_mqtt_client_publish(s_client, s_topic_pressure, pressure_str,
                                               0, ADAFRUIT_MQTT_QOS, 0);
    int msg_humidity = 0;

    if (has_humidity && s_has_humidity_feed) {
        snprintf(humidity_str, sizeof(humidity_str), "%.2f", humidity_percent);
        msg_humidity = esp_mqtt_client_publish(s_client, s_topic_humidity, humidity_str,
                                               0, ADAFRUIT_MQTT_QOS, 0);
    }

    if (msg_temp < 0 || msg_pressure < 0 || msg_humidity < 0) {
        ESP_LOGE(TAG, "Publish failed");
        return ESP_FAIL;
    }

    if (has_humidity && s_has_humidity_feed) {
        ESP_LOGI(TAG, "Uploaded -> temp=%s pressure=%s humidity=%s",
                 temp_str, pressure_str, humidity_str);
    } else {
        ESP_LOGI(TAG, "Uploaded -> temp=%s pressure=%s", temp_str, pressure_str);
    }
    return ESP_OK;
}

static void flush_pending(void)
{
    if (!s_pending.valid || !s_connected) {
        return;
    }

    ESP_LOGI(TAG, "Flushing buffered sample");
    if (publish_values(s_pending.temperature_c, s_pending.pressure_hpa,
                       s_pending.humidity_percent, s_pending.has_humidity) == ESP_OK) {
        s_pending.valid = false;
    }
}

static void store_pending(float temperature_c,
                          float pressure_hpa,
                          float humidity_percent,
                          bool has_humidity)
{
    s_pending.temperature_c = temperature_c;
    s_pending.pressure_hpa = pressure_hpa;
    s_pending.humidity_percent = humidity_percent;
    s_pending.has_humidity = has_humidity;
    s_pending.valid = true;
    ESP_LOGW(TAG, "Buffered sample for later publish");
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data)
{
    (void)handler_args;
    (void)base;
    esp_mqtt_event_handle_t event = event_data;

    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        s_connected = true;
        xEventGroupSetBits(s_mqtt_events, MQTT_CONNECTED_BIT);
        ESP_LOGI(TAG, "Connected to Adafruit IO (TLS)");
        esp_mqtt_client_publish(s_client, s_topic_status, "online",
                                0, ADAFRUIT_MQTT_QOS, 1);
        flush_pending();
        break;
    case MQTT_EVENT_DISCONNECTED:
        s_connected = false;
        xEventGroupClearBits(s_mqtt_events, MQTT_CONNECTED_BIT);
        ESP_LOGW(TAG, "Disconnected from Adafruit IO");
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "MQTT error");
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGD(TAG, "Publish ok, msg_id=%d", event->msg_id);
        break;
    default:
        break;
    }
}

esp_err_t adafruit_mqtt_start(const adafruit_feeds_t *feeds)
{
    if (feeds == NULL || feeds->feed_temp == NULL || feeds->feed_pressure == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    build_topics(feeds);

    s_mqtt_events = xEventGroupCreate();
    if (s_mqtt_events == NULL) {
        return ESP_ERR_NO_MEM;
    }

    static char client_id[64];
    static char broker_uri[80];
    snprintf(client_id, sizeof(client_id), "esp32-%s", feeds->feed_temp);
    snprintf(broker_uri, sizeof(broker_uri), "mqtts://%s:%d",
             ADAFRUIT_MQTT_HOST, ADAFRUIT_MQTT_PORT_TLS);

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = broker_uri,
        .broker.verification.crt_bundle_attach = esp_crt_bundle_attach,
        .credentials.username = ADAFRUIT_USERNAME,
        .credentials.authentication.password = ADAFRUIT_AIO_KEY,
        .credentials.client_id = client_id,
        .session.keepalive = 60,
        .session.last_will.topic = s_topic_status,
        .session.last_will.msg = "offline",
        .session.last_will.msg_len = 7,
        .session.last_will.qos = ADAFRUIT_MQTT_QOS,
        .session.last_will.retain = true,
        .network.reconnect_timeout_ms = 10000,
    };

    s_client = esp_mqtt_client_init(&mqtt_cfg);
    if (s_client == NULL) {
        ESP_LOGE(TAG, "MQTT client init failed");
        return ESP_FAIL;
    }

    esp_mqtt_client_register_event(s_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_err_t err = esp_mqtt_client_start(s_client);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MQTT start failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "MQTT starting (mqtts://%s:%d)...",
             ADAFRUIT_MQTT_HOST, ADAFRUIT_MQTT_PORT_TLS);
    ESP_LOGI(TAG, "Temp feed:     %s", s_topic_temp);
    ESP_LOGI(TAG, "Pressure feed: %s", s_topic_pressure);
    if (s_has_humidity_feed) {
        ESP_LOGI(TAG, "Humidity feed: %s", s_topic_humidity);
    }
    ESP_LOGI(TAG, "Status feed:   %s", s_topic_status);
    return ESP_OK;
}

esp_err_t adafruit_mqtt_wait_connected(TickType_t ticks_to_wait)
{
    if (s_mqtt_events == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    EventBits_t bits = xEventGroupWaitBits(s_mqtt_events, MQTT_CONNECTED_BIT,
                                           pdFALSE, pdFALSE, ticks_to_wait);
    return (bits & MQTT_CONNECTED_BIT) ? ESP_OK : ESP_ERR_TIMEOUT;
}

bool adafruit_mqtt_is_connected(void)
{
    return s_connected;
}

esp_err_t adafruit_mqtt_publish_sensor(float temperature_c,
                                       float pressure_hpa,
                                       float humidity_percent,
                                       bool has_humidity)
{
    if (s_client == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!wifi_is_connected() || !s_connected) {
        store_pending(temperature_c, pressure_hpa, humidity_percent, has_humidity);
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = publish_values(temperature_c, pressure_hpa,
                                   humidity_percent, has_humidity);
    if (err != ESP_OK) {
        store_pending(temperature_c, pressure_hpa, humidity_percent, has_humidity);
    }
    return err;
}
