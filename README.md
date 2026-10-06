# ESP32 Sensor Cloud MQTT

ESP32 firmware that reads BMP180 or BMP/BME280 over I2C and publishes temperature, pressure (and humidity when available) to [Adafruit IO](https://io.adafruit.com) over **MQTT TLS**.

## Features

- Runtime sensor auto-detect (BMP180 / BMP280 / BME280), or force via `SENSOR_TYPE`
- Secrets kept in local `main/secrets.h` (gitignored)
- MQTT over TLS (`mqtts://io.adafruit.com:8883`) with certificate bundle
- QoS 1 publishes, latest-sample buffer while offline, flush on reconnect
- Wi-Fi reconnect with exponential backoff
- Device status feed + MQTT Last Will (`online` / `offline`)
- LED patterns for Wi-Fi wait / MQTT wait / sensor fail / publish fail / OK
- Optional deep sleep between samples (`ENABLE_DEEP_SLEEP`)
- Task watchdog on the sampling loop
- Display ESP32 (`SENSOR_DASHBOARD`): no sensor, subscribes to both boards' feeds and serves a local web page

## Hardware

4-pin BMP180 / BMP280 / BME280 modules (Vin, GND, SDA, SCL) use I2C only — no CS/SDO/SPI:

| Module | ESP32 |
|--------|-------|
| Vin    | 3.3V |
| GND    | GND |
| SDA    | GPIO21 (D21) |
| SCL    | GPIO22 (D22) |
| LED (onboard) | GPIO2 |

## Setup

1. Copy credentials template and edit:

   ```text
   copy main\secrets.h.example main\secrets.h
   ```

   Fill in `WIFI_SSID`, `WIFI_PASSWORD`, `ADAFRUIT_USERNAME`, `ADAFRUIT_AIO_KEY`.

2. Create Adafruit IO feeds (exact names):

   - BMP180: `bmp180-temp`, `bmp180-pressure`
   - BMP/BME280: `bmp280-temp`, `bmp280-pressure`, `bmp280-humidity`
   - Status (both boards): `device-status`

3. If an AIO key was ever committed or shared, **rotate it** in Adafruit IO → My Key.

4. Build and flash with ESP-IDF:

   ```bash
   idf.py set-target esp32
   idf.py build flash monitor
   ```

## Configuration (`main/config.h`)

| Macro | Meaning |
|-------|---------|
| `SENSOR_TYPE` | `SENSOR_AUTO` (sensor boards), `SENSOR_BMP180`, `SENSOR_BMP_BME280`, or `SENSOR_DASHBOARD` (display board) |
| `SENSOR_READ_MS` | Sample / upload period (default 10000) |
| `ENABLE_DEEP_SLEEP` | `1` = sleep between samples; `0` = always-on task |
| `WEB_SERVER_PORT` | Display board HTTP port (default 80) |
| `WEB_PAGE_REFRESH_SEC` | How often the display page reloads (default 5) |
| `WIFI_USE_STATIC_IP` | `1` on display board; uses `WIFI_STATIC_IP` / gateway / netmask / DNS |

## Display board (third ESP32, no sensor)

1. In `main/config.h` set `SENSOR_TYPE` to `SENSOR_DASHBOARD`.
2. Build and flash that board only. Sensor boards must be built with `SENSOR_AUTO` (or a forced sensor type).
3. After boot, the serial log prints `Web page: http://<ip>/`. Open that address from a phone or PC on the same Wi-Fi.

The page is one table:

| Sensor | Temperature | Pressure | Humidity |
|--------|-------------|----------|----------|
| BME280 Sensor Data | from `bmp280-temp` | from `bmp280-pressure` | from `bmp280-humidity` |
| BMP180 Sensor Data | from `bmp180-temp` | from `bmp180-pressure` | not available |

The BME280 row uses the existing `bmp280-*` feeds. Cells stay `--` until Adafruit IO has a value. The page reloads on its own.

## LED patterns

| Pattern | Meaning |
|---------|---------|
| Slow pulse | Waiting for Wi-Fi |
| Two short blinks | Waiting for MQTT |
| Three short blinks | Sensor read/init failure |
| Long blink | Publish failed / buffered |
| One short blink | Sample published |

## Security note

Never commit `main/secrets.h`. Use TLS (enabled by default). Prefer rotating any key that appeared in an old `config.h` commit.
