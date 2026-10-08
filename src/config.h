#pragma once

/* ----------------------------- WiFi ----------------------------- */

#define WIFI_SSID     "WifiDev"
#define WIFI_PASSWORD "rpidev24"
#define WIFI_CONNECT_TIMEOUT_MS 15000

/* ----------------------- AquaControl Dashboard ----------------------- */

#define AQUACONTROL_HOST       "tesisutec.vercel.app"
#define AQUACONTROL_PATH       "/api/telemetry/sensor_bulk"
#define AQUACONTROL_DEVICE_KEY "ESP32_OD_SENSOR"

/* ------------------------------ BLE ---------------------------------- */

#define SERVICE_UUID               "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID        "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHARACTERISTIC_NOTIFY_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a9"

#define BLE_ACTIVE_TIME 30000

/* ------------------------------ Timezone ------------------------------ */

#define LOCAL_TZ_OFFSET_SEC (5 * 3600) // UTC-5 (Peru / PET)
#define LOCAL_TZ_NAME       "UTC-5"
