#pragma once

/* ------------------------------- Wi-Fi ---------------------------------- */
#define WIFI_SSID       "YOUR_SSID"
#define WIFI_PASSWORD   "YOUR_PASSWORD"

/* ------------------------------ Network --------------------------------- */
#define SERVER_PORT             6969
#define HANDSHAKE_RETRY_MS      500
#define HANDSHAKE_MAGIC         "ZT_SERVER_HELLO"

/* ------------------------------- I2C ------------------------------------ */
#define I2C_SPEED 1'000'000 // 1MHZ

/* ------------------------------- IMU ------------------------------------ */
#define IMU_I2C_ADDRESS         0x6Au       /* LSM6DSR SA0 = 0 */
#define PIN_IMU_SDA             4u          /* ESP8266 GPIO4 */
#define PIN_IMU_SCL             5u          /* ESP8266 GPIO5 */

/* ------------------------------ Serial ---------------------------------- */
#define SERIAL_BAUD_RATE 115200

/* ------------------------------- Other --------------------------------- */
#define LED_PIN                 2u
#define LED_INVERTED            0
#define PIN_BATTERY_LEVEL       0u
