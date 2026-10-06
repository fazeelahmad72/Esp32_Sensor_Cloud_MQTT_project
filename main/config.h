#pragma once

#include "secrets.h"

/*
 * Board selection (set this before flashing each ESP32):
 *   SENSOR_AUTO       -> probe I2C chip ID at runtime (sensor boards)
 *   SENSOR_BMP180     -> force BMP180 feeds/driver
 *   SENSOR_BMP_BME280 -> force BMP/BME280 feeds/driver
 *   SENSOR_DASHBOARD  -> no sensor; read Adafruit IO and serve a web page
 *
 * Display board is selected below. Sensor boards must be built with SENSOR_AUTO.
 */
#define SENSOR_AUTO        0
#define SENSOR_BMP180      1
#define SENSOR_BMP280      2
#define SENSOR_BMP_BME280  SENSOR_BMP280
#define SENSOR_DASHBOARD   3

#ifndef SENSOR_TYPE
#define SENSOR_TYPE        SENSOR_DASHBOARD
#endif

/*
 * I2C pins for 4-wire BMP180 / BMP280 / BME280 modules (Vin, GND, SDA, SCL):
 *   Vin -> 3.3V, GND -> GND, SDA -> D21 (GPIO21), SCL -> D22 (GPIO22)
 * No CS / SDO / SPI wiring is required in software.
 */
#define I2C_SDA_PIN    21
#define I2C_SCL_PIN    22
#define I2C_FREQ_HZ    100000

/* Onboard LED: D02 = GPIO2 */
#define LED_PIN            2
#define LED_BLINK_MS       100

/*
 * How often to read + upload (milliseconds).
 * Adafruit IO free plan allows ~30 points/minute.
 * Three feeds every 10s = 18 points/minute (safe).
 */
#define SENSOR_READ_MS     10000

/* 1 = deep sleep between samples (battery-friendly). 0 = always-on task loop. */
#define ENABLE_DEEP_SLEEP  0

/* Display ESP32 only: local page that shows both sensors from Adafruit IO.
 * WEB_PAGE_REFRESH_SEC is how often the browser polls /api/status. */
#define WEB_SERVER_PORT       80
#define WEB_PAGE_REFRESH_SEC  2

/*
 * Static IP for the display board (SENSOR_DASHBOARD).
 * Sensor boards keep DHCP unless you also set WIFI_USE_STATIC_IP to 1 for them.
 */
#if SENSOR_TYPE == SENSOR_DASHBOARD
#define WIFI_USE_STATIC_IP    1
#else
#define WIFI_USE_STATIC_IP    0
#endif
/* Must match your router LAN (usually 192.168.1.x, not 195.x). */
#define WIFI_STATIC_IP        "192.168.1.15"
#define WIFI_STATIC_GATEWAY   "192.168.1.1"
#define WIFI_STATIC_NETMASK   "255.255.255.0"
#define WIFI_STATIC_DNS       "192.168.1.1"
#define WIFI_STATIC_DNS_FALLBACK "8.8.8.8"

/* ===================== Adafruit IO ===================== */
#define ADAFRUIT_MQTT_HOST "io.adafruit.com"
#define ADAFRUIT_MQTT_PORT_TLS 8883
#define ADAFRUIT_MQTT_QOS  1

/* Create this feed in Adafruit IO for online/offline Last Will */
#define ADAFRUIT_FEED_STATUS "device-status"

/*
 * Create these feeds in Adafruit IO (exact names):
 *   BMP180 board:     bmp180-temp , bmp180-pressure
 *   BMP/BME280 board: bmp280-temp , bmp280-pressure , bmp280-humidity
 */
#define ADAFRUIT_FEED_BMP180_TEMP      "bmp180-temp"
#define ADAFRUIT_FEED_BMP180_PRESSURE  "bmp180-pressure"
#define ADAFRUIT_FEED_BMP280_TEMP      "bmp280-temp"
#define ADAFRUIT_FEED_BMP280_PRESSURE  "bmp280-pressure"
#define ADAFRUIT_FEED_BMP280_HUMIDITY  "bmp280-humidity"
