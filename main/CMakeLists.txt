idf_component_register(
    SRCS
        "main.c"
        "i2c_bus.c"
        "bmp180_sensor.c"
        "bmp280_sensor.c"
        "led_blink.c"
        "wifi_connect.c"
        "adafruit_mqtt.c"
        "cloud_dashboard.c"
    INCLUDE_DIRS "."
    EMBED_FILES "dashboard.html"
    REQUIRES
        driver
        freertos
        log
        esp_rom
        nvs_flash
        esp_wifi
        esp_event
        esp_netif
        mqtt
        mbedtls
        esp_hw_support
        esp_http_server
        esp_http_client
        json
)
