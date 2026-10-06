#include "cloud_dashboard.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cJSON.h"
#include "config.h"
#include "esp_crt_bundle.h"
#include "esp_heap_caps.h"
#include "esp_http_client.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "mqtt_client.h"

static const char *TAG = "dashboard";

extern const uint8_t dashboard_html_start[] asm("_binary_dashboard_html_start");
extern const uint8_t dashboard_html_end[] asm("_binary_dashboard_html_end");

#define MQTT_CONNECTED_BIT BIT0
#define FEED_COUNT         5
#define VALUE_LEN          24
#define HIST_CAP           40
#define HIST_COALESCE_MS   8000

enum {
    FEED_BME_TEMP = 0,
    FEED_BME_PRESSURE,
    FEED_BME_HUMIDITY,
    FEED_BMP_TEMP,
    FEED_BMP_PRESSURE,
};

typedef struct {
    char topic[96];
    char value[VALUE_LEN];
    bool valid;
    uint32_t updated_ticks;
} cloud_feed_t;

static esp_mqtt_client_handle_t s_client = NULL;
static httpd_handle_t s_httpd = NULL;
static EventGroupHandle_t s_mqtt_events;
static SemaphoreHandle_t s_lock;
static bool s_connected = false;
static int s_subscribed = 0;
static bool s_latest_requested = false;
static volatile bool s_need_mqtt_get = false;
static cloud_feed_t s_feeds[FEED_COUNT];

typedef struct {
    float value[FEED_COUNT];
    uint8_t mask;
    uint32_t ticks;
} hist_sample_t;

static hist_sample_t s_hist[HIST_CAP];
static int s_hist_len;
static int s_hist_pos;
static uint32_t s_hist_last_ticks;

static const char *s_feed_keys[FEED_COUNT] = {
    ADAFRUIT_FEED_BMP280_TEMP,
    ADAFRUIT_FEED_BMP280_PRESSURE,
    ADAFRUIT_FEED_BMP280_HUMIDITY,
    ADAFRUIT_FEED_BMP180_TEMP,
    ADAFRUIT_FEED_BMP180_PRESSURE,
};

static bool number_ok(const char *s)
{
    if (s == NULL || *s == '\0') {
        return false;
    }
    if (*s == '-' || *s == '+') {
        s++;
    }
    bool digit = false;
    bool dot = false;
    for (; *s != '\0'; s++) {
        if (*s >= '0' && *s <= '9') {
            digit = true;
            continue;
        }
        if (*s == '.' && !dot) {
            dot = true;
            continue;
        }
        return false;
    }
    return digit;
}

static void copy_payload(char *dst, size_t dst_len, const char *src, int src_len)
{
    if (src == NULL || src_len < 0) {
        src_len = 0;
    }
    while (src_len > 0 && (src[0] == ' ' || src[0] == '\r' || src[0] == '\n')) {
        src++;
        src_len--;
    }
    while (src_len > 0 &&
           (src[src_len - 1] == ' ' || src[src_len - 1] == '\r' || src[src_len - 1] == '\n')) {
        src_len--;
    }
    if ((size_t)src_len >= dst_len) {
        src_len = (int)dst_len - 1;
    }
    if (src_len > 0) {
        memcpy(dst, src, (size_t)src_len);
    }
    dst[src_len] = '\0';
}

static int feed_index_for_topic(const char *topic, int topic_len)
{
    if (topic == NULL || topic_len <= 0) {
        return -1;
    }
    for (int i = 0; i < FEED_COUNT; i++) {
        size_t n = strlen(s_feeds[i].topic);
        if ((int)n == topic_len && memcmp(s_feeds[i].topic, topic, n) == 0) {
            return i;
        }
    }
    return -1;
}

static bool read_feed(int index, float *out)
{
    if (index < 0 || index >= FEED_COUNT || !s_feeds[index].valid ||
        !number_ok(s_feeds[index].value)) {
        return false;
    }
    *out = strtof(s_feeds[index].value, NULL);
    return true;
}

static uint32_t age_seconds(int index, uint32_t now)
{
    if (!s_feeds[index].valid) {
        return UINT32_MAX;
    }
    return ((now - s_feeds[index].updated_ticks) * portTICK_PERIOD_MS) / 1000;
}

static uint32_t younger(uint32_t a, uint32_t b)
{
    return (a < b) ? a : b;
}

typedef struct {
    char *buf;
    int len;
    int cap;
} http_buf_t;

static esp_err_t http_event_handler(esp_http_client_event_t *evt)
{
    http_buf_t *hb = evt->user_data;
    if (evt->event_id != HTTP_EVENT_ON_DATA || hb == NULL || evt->data == NULL) {
        return ESP_OK;
    }
    if (hb->len + evt->data_len + 1 > hb->cap) {
        return ESP_OK;
    }
    memcpy(hb->buf + hb->len, evt->data, evt->data_len);
    hb->len += evt->data_len;
    hb->buf[hb->len] = '\0';
    return ESP_OK;
}

/* Merge feed updates that arrive together into one chart sample. */
static void hist_note_locked(int index, float value)
{
    uint32_t now = xTaskGetTickCount();
    bool start_new = (s_hist_len == 0) ||
                     ((now - s_hist_last_ticks) >= pdMS_TO_TICKS(HIST_COALESCE_MS));
    hist_sample_t *sample;

    if (start_new) {
        sample = &s_hist[s_hist_pos];
        if (s_hist_len > 0) {
            int prev = (s_hist_pos + HIST_CAP - 1) % HIST_CAP;
            *sample = s_hist[prev];
        } else {
            memset(sample, 0, sizeof(*sample));
        }
        s_hist_pos = (s_hist_pos + 1) % HIST_CAP;
        if (s_hist_len < HIST_CAP) {
            s_hist_len++;
        }
    } else {
        int last = (s_hist_pos + HIST_CAP - 1) % HIST_CAP;
        sample = &s_hist[last];
    }

    sample->ticks = now;
    sample->value[index] = value;
    sample->mask |= (uint8_t)(1u << index);
    s_hist_last_ticks = now;
}

static int hist_chron(int i)
{
    int start = (s_hist_len < HIST_CAP) ? 0 : s_hist_pos;
    return (start + i) % HIST_CAP;
}

static bool store_feed_value(int index, const char *value)
{
    if (index < 0 || index >= FEED_COUNT || value == NULL || !number_ok(value)) {
        return false;
    }
    float number = strtof(value, NULL);
    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(100)) != pdTRUE) {
        return false;
    }
    snprintf(s_feeds[index].value, sizeof(s_feeds[index].value), "%s", value);
    s_feeds[index].valid = true;
    s_feeds[index].updated_ticks = xTaskGetTickCount();
    hist_note_locked(index, number);
    xSemaphoreGive(s_lock);
    return true;
}

/* One TLS handshake needs a large contiguous block. Skip it when the heap
 * is already fragmented so the web server can still accept browsers. */
static bool tls_heap_ok(void)
{
    size_t largest = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
    if (largest < 36 * 1024) {
        ESP_LOGW(TAG, "Skip HTTPS fetch, largest free block %u", (unsigned)largest);
        return false;
    }
    return true;
}

/* REST fallback when MQTT is down. Do not call this in a tight loop. */
static void fetch_feed_last_http(int index)
{
    if (!tls_heap_ok()) {
        return;
    }

    char url[160];
    char body[384];
    http_buf_t hb = { .buf = body, .len = 0, .cap = (int)sizeof(body) };
    body[0] = '\0';

    snprintf(url, sizeof(url),
             "https://io.adafruit.com/api/v2/%s/feeds/%s/data/last",
             ADAFRUIT_USERNAME, s_feed_keys[index]);

    esp_http_client_config_t cfg = {
        .url = url,
        .method = HTTP_METHOD_GET,
        .timeout_ms = 6000,
        .buffer_size = 1024,
        .buffer_size_tx = 512,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .event_handler = http_event_handler,
        .user_data = &hb,
    };

    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (client == NULL) {
        return;
    }
    esp_http_client_set_header(client, "X-AIO-Key", ADAFRUIT_AIO_KEY);
    esp_http_client_set_header(client, "Accept", "application/json");
    esp_http_client_set_header(client, "Connection", "close");

    esp_err_t err = esp_http_client_perform(client);
    int status = esp_http_client_get_status_code(client);
    esp_http_client_close(client);
    esp_http_client_cleanup(client);

    if (err != ESP_OK || status != 200 || hb.len <= 0) {
        ESP_LOGW(TAG, "HTTP last %s -> err=%s status=%d",
                 s_feed_keys[index], esp_err_to_name(err), status);
        return;
    }

    cJSON *root = cJSON_Parse(body);
    if (root == NULL) {
        return;
    }
    cJSON *value = cJSON_GetObjectItem(root, "value");
    if (cJSON_IsString(value) && value->valuestring != NULL) {
        if (store_feed_value(index, value->valuestring)) {
            ESP_LOGI(TAG, "HTTP %s = %s", s_feed_keys[index], value->valuestring);
        }
    }
    cJSON_Delete(root);
}

/* Ask Adafruit IO MQTT to republish the last stored value for each feed. */
static void request_latest_mqtt(void)
{
    if (!s_connected || s_client == NULL) {
        return;
    }
    for (int i = 0; i < FEED_COUNT; i++) {
        char get_topic[128];
        snprintf(get_topic, sizeof(get_topic), "%s/get", s_feeds[i].topic);
        int msg_id = esp_mqtt_client_publish(s_client, get_topic, "\0", 1, 0, 0);
        if (msg_id < 0) {
            ESP_LOGW(TAG, "MQTT get failed: %s", s_feed_keys[i]);
        }
        vTaskDelay(pdMS_TO_TICKS(120));
    }
}

static void fetch_all_http(void)
{
    for (int i = 0; i < FEED_COUNT; i++) {
        fetch_feed_last_http(i);
        vTaskDelay(pdMS_TO_TICKS(300));
    }
}

/* Live values come from the existing MQTT session. HTTPS is only a slow
 * fallback: a TLS socket per feed every 15s filled the TCP pool (TIME_WAIT)
 * and fragmented the heap until the browser could not open the page. */
static void dashboard_maint_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(2000));

    uint32_t last_http_ticks = 0;
    uint32_t mqtt_down_since = xTaskGetTickCount();
    uint32_t last_log = 0;
    bool was_up = false;
    bool http_ever = false;

    while (1) {
        uint32_t now = xTaskGetTickCount();
        bool up = s_connected;

        if (up) {
            was_up = true;
            mqtt_down_since = now;
            if (s_need_mqtt_get) {
                s_need_mqtt_get = false;
                request_latest_mqtt();
                ESP_LOGI(TAG, "Requested last values over MQTT");
            }
        } else {
            if (was_up) {
                mqtt_down_since = now;
                was_up = false;
            }
            bool http_due = !http_ever ||
                            (now - last_http_ticks) >= pdMS_TO_TICKS(120000);
            if (http_due && (now - mqtt_down_since) >= pdMS_TO_TICKS(20000)) {
                ESP_LOGW(TAG, "MQTT down, refreshing feeds over HTTPS");
                fetch_all_http();
                http_ever = true;
                last_http_ticks = xTaskGetTickCount();
            }
        }

        if ((now - last_log) >= pdMS_TO_TICKS(60000)) {
            last_log = now;
            ESP_LOGI(TAG, "heap free=%u min=%u largest=%u mqtt=%d",
                     (unsigned)esp_get_free_heap_size(),
                     (unsigned)esp_get_minimum_free_heap_size(),
                     (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT),
                     up ? 1 : 0);
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
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
        s_subscribed = 0;
        s_latest_requested = false;
        xEventGroupSetBits(s_mqtt_events, MQTT_CONNECTED_BIT);
        ESP_LOGI(TAG, "Connected to Adafruit IO (TLS)");
        for (int i = 0; i < FEED_COUNT; i++) {
            int msg_id = esp_mqtt_client_subscribe_single(s_client, s_feeds[i].topic, 1);
            if (msg_id < 0) {
                ESP_LOGE(TAG, "Subscribe failed: %s", s_feeds[i].topic);
            }
        }
        break;
    case MQTT_EVENT_DISCONNECTED:
        s_connected = false;
        xEventGroupClearBits(s_mqtt_events, MQTT_CONNECTED_BIT);
        ESP_LOGW(TAG, "Disconnected from Adafruit IO");
        break;
    case MQTT_EVENT_SUBSCRIBED:
        s_subscribed++;
        ESP_LOGI(TAG, "Subscribed %d/%d, msg_id=%d", s_subscribed, FEED_COUNT, event->msg_id);
        if (s_subscribed >= FEED_COUNT && !s_latest_requested) {
            s_latest_requested = true;
            s_need_mqtt_get = true;
        }
        break;
    case MQTT_EVENT_DATA: {
        int index = feed_index_for_topic(event->topic, event->topic_len);
        if (index < 0 || event->data == NULL) {
            break;
        }
        char payload[VALUE_LEN];
        copy_payload(payload, sizeof(payload), event->data, event->data_len);
        if (store_feed_value(index, payload)) {
            ESP_LOGI(TAG, "MQTT %s = %s", s_feed_keys[index], payload);
        }
        break;
    }
    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "MQTT error");
        break;
    default:
        break;
    }
}

typedef struct {
    bool mqtt;
    float value[FEED_COUNT];
    bool ok[FEED_COUNT];
    uint32_t bme_age;
    uint32_t bmp_age;
    bool bme_any;
    bool bmp_any;
    hist_sample_t hist[HIST_CAP];
    int hist_len;
    uint32_t now;
} dash_snap_t;

static void fill_snap(dash_snap_t *snap)
{
    snap->mqtt = s_connected;
    snap->now = xTaskGetTickCount();
    for (int i = 0; i < FEED_COUNT; i++) {
        snap->ok[i] = read_feed(i, &snap->value[i]);
    }
    snap->bme_any = snap->ok[FEED_BME_TEMP] || snap->ok[FEED_BME_PRESSURE] ||
                    snap->ok[FEED_BME_HUMIDITY];
    snap->bmp_any = snap->ok[FEED_BMP_TEMP] || snap->ok[FEED_BMP_PRESSURE];
    snap->bme_age = younger(age_seconds(FEED_BME_TEMP, snap->now),
                             younger(age_seconds(FEED_BME_PRESSURE, snap->now),
                                     age_seconds(FEED_BME_HUMIDITY, snap->now)));
    snap->bmp_age = younger(age_seconds(FEED_BMP_TEMP, snap->now),
                             age_seconds(FEED_BMP_PRESSURE, snap->now));
    snap->hist_len = s_hist_len;
    for (int i = 0; i < s_hist_len; i++) {
        snap->hist[i] = s_hist[hist_chron(i)];
    }
}

static void json_num(cJSON *obj, const char *key, bool ok, float value)
{
    if (obj == NULL) {
        return;
    }
    if (ok) {
        cJSON_AddNumberToObject(obj, key, (double)value);
    } else {
        cJSON_AddNullToObject(obj, key);
    }
}

static void json_age(cJSON *obj, bool any, uint32_t age)
{
    if (obj == NULL) {
        return;
    }
    if (!any || age == UINT32_MAX) {
        cJSON_AddNullToObject(obj, "age");
    } else {
        cJSON_AddNumberToObject(obj, "age", (double)age);
    }
}

static cJSON *snap_to_json(const dash_snap_t *snap)
{
    static const char *keys[FEED_COUNT] = { "t", "p", "h", "T", "P" };
    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
        return NULL;
    }

    cJSON_AddNumberToObject(root, "poll", WEB_PAGE_REFRESH_SEC);
    cJSON_AddBoolToObject(root, "mqtt", snap->mqtt);

    cJSON *bme = cJSON_AddObjectToObject(root, "bme");
    json_num(bme, "temp", snap->ok[FEED_BME_TEMP], snap->value[FEED_BME_TEMP]);
    json_num(bme, "pressure", snap->ok[FEED_BME_PRESSURE], snap->value[FEED_BME_PRESSURE]);
    json_num(bme, "humidity", snap->ok[FEED_BME_HUMIDITY], snap->value[FEED_BME_HUMIDITY]);
    json_age(bme, snap->bme_any, snap->bme_age);

    cJSON *bmp = cJSON_AddObjectToObject(root, "bmp");
    json_num(bmp, "temp", snap->ok[FEED_BMP_TEMP], snap->value[FEED_BMP_TEMP]);
    json_num(bmp, "pressure", snap->ok[FEED_BMP_PRESSURE], snap->value[FEED_BMP_PRESSURE]);
    json_age(bmp, snap->bmp_any, snap->bmp_age);

    cJSON *hist = cJSON_AddArrayToObject(root, "hist");
    for (int i = 0; i < snap->hist_len && hist != NULL; i++) {
        const hist_sample_t *sample = &snap->hist[i];
        cJSON *item = cJSON_CreateObject();
        if (item == NULL) {
            break;
        }
        uint32_t age = ((snap->now - sample->ticks) * portTICK_PERIOD_MS) / 1000;
        cJSON_AddNumberToObject(item, "a", (double)age);
        for (int f = 0; f < FEED_COUNT; f++) {
            json_num(item, keys[f], (sample->mask & (uint8_t)(1u << f)) != 0, sample->value[f]);
        }
        cJSON_AddItemToArray(hist, item);
    }
    return root;
}

static esp_err_t api_status_handler(httpd_req_t *req)
{
    dash_snap_t *snap = calloc(1, sizeof(*snap));
    if (snap == NULL) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no mem");
        return ESP_ERR_NO_MEM;
    }
    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(200)) != pdTRUE) {
        free(snap);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "busy");
        return ESP_FAIL;
    }
    fill_snap(snap);
    xSemaphoreGive(s_lock);

    cJSON *root = snap_to_json(snap);
    free(snap);
    if (root == NULL) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no mem");
        return ESP_ERR_NO_MEM;
    }

    char *printed = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (printed == NULL) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no mem");
        return ESP_ERR_NO_MEM;
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t err = httpd_resp_send(req, printed, HTTPD_RESP_USE_STRLEN);
    cJSON_free(printed);
    return err;
}

static esp_err_t root_get_handler(httpd_req_t *req)
{
    const size_t len = (size_t)(dashboard_html_end - dashboard_html_start);
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, (const char *)dashboard_html_start, (ssize_t)len);
}

static esp_err_t favicon_get_handler(httpd_req_t *req)
{
    httpd_resp_set_status(req, "204 No Content");
    return httpd_resp_send(req, "", 0);
}

static esp_err_t register_get(const char *uri, esp_err_t (*handler)(httpd_req_t *))
{
    httpd_uri_t route = {
        .uri = uri,
        .method = HTTP_GET,
        .handler = handler,
        .user_ctx = NULL,
    };
    return httpd_register_uri_handler(s_httpd, &route);
}

static esp_err_t start_web_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = WEB_SERVER_PORT;
    config.lru_purge_enable = true;
    /* 4 browser sockets + 3 internal. Leaves lwIP sockets for MQTT. */
    config.max_open_sockets = 4;
    config.max_uri_handlers = 8;
    config.backlog_conn = 2;
    config.stack_size = 8192;
    config.recv_wait_timeout = 5;
    config.send_wait_timeout = 5;
    /* RST on close so idle browsers do not park TCP blocks in TIME_WAIT. */
    config.enable_so_linger = true;
    config.linger_timeout = 0;

    esp_err_t err = httpd_start(&s_httpd, &config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "HTTP server start failed: %s", esp_err_to_name(err));
        return err;
    }

    err = register_get("/", root_get_handler);
    if (err == ESP_OK) {
        err = register_get("/api/status", api_status_handler);
    }
    if (err == ESP_OK) {
        err = register_get("/favicon.ico", favicon_get_handler);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "HTTP handler register failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "Web server on port %d", WEB_SERVER_PORT);
    return ESP_OK;
}

static esp_err_t start_mqtt(void)
{
    for (int i = 0; i < FEED_COUNT; i++) {
        snprintf(s_feeds[i].topic, sizeof(s_feeds[i].topic), "%s/feeds/%s",
                 ADAFRUIT_USERNAME, s_feed_keys[i]);
        s_feeds[i].value[0] = '\0';
        s_feeds[i].valid = false;
        ESP_LOGI(TAG, "Feed: %s", s_feeds[i].topic);
    }

    s_mqtt_events = xEventGroupCreate();
    if (s_mqtt_events == NULL || s_lock == NULL) {
        return ESP_ERR_NO_MEM;
    }

    static char broker_uri[80];
    snprintf(broker_uri, sizeof(broker_uri), "mqtts://%s:%d",
             ADAFRUIT_MQTT_HOST, ADAFRUIT_MQTT_PORT_TLS);

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = broker_uri,
        .broker.verification.crt_bundle_attach = esp_crt_bundle_attach,
        .credentials.username = ADAFRUIT_USERNAME,
        .credentials.authentication.password = ADAFRUIT_AIO_KEY,
        .credentials.client_id = "esp32-dashboard",
        .session.keepalive = 60,
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
    return ESP_OK;
}

esp_err_t cloud_dashboard_start(void)
{
    s_lock = xSemaphoreCreateMutex();
    if (s_lock == NULL) {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = start_web_server();
    if (err != ESP_OK) {
        return err;
    }
    err = start_mqtt();
    if (err != ESP_OK) {
        return err;
    }

    if (xTaskCreate(dashboard_maint_task, "dash_maint", 10240, NULL, 4, NULL) != pdPASS) {
        ESP_LOGW(TAG, "Dashboard maintenance task not started");
    }
    return ESP_OK;
}

esp_err_t cloud_dashboard_wait_connected(TickType_t ticks_to_wait)
{
    if (s_mqtt_events == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    EventBits_t bits = xEventGroupWaitBits(s_mqtt_events, MQTT_CONNECTED_BIT,
                                           pdFALSE, pdFALSE, ticks_to_wait);
    return (bits & MQTT_CONNECTED_BIT) ? ESP_OK : ESP_ERR_TIMEOUT;
}
