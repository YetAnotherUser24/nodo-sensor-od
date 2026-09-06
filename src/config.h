#pragma once

/* ----------------------------- WiFi ----------------------------- */

#define WIFI_SSID     "WifiDev"
#define WIFI_PASSWORD "rpidev24"
#define WIFI_CONNECT_TIMEOUT_MS 15000

/* ----------------------- AquaControl Dashboard ----------------------- */

#define AQUACONTROL_HOST       "tesisutec.vercel.app"
#define AQUACONTROL_PATH       "/api/telemetry/sensor"
#define AQUACONTROL_DEVICE_KEY "ESP32_OD_SENSOR"

/* ------------------------------ BLE ---------------------------------- */

#define SERVICE_UUID               "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID        "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHARACTERISTIC_NOTIFY_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a9"

#define BLE_ACTIVE_TIME 30000
