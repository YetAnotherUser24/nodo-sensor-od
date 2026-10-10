#pragma once

/* ----------------------------- WiFi ----------------------------- */

#define WIFI_SSID     "WifiDev"
#define WIFI_PASSWORD "rpidev24"
#define WIFI_CONNECT_TIMEOUT_MS 15000

/* ----------------------- AquaControl Dashboard ----------------------- */

#define AQUACONTROL_HOST       "tesisutec.vercel.app"
#define AQUACONTROL_PATH       "/api/telemetry/sensor_bulk"
#define AQUACONTROL_DEVICE_KEY "ESP32_OD_SENSOR"

/* ------------------- AquaControl MQTT (schema v2) ------------------ */
/*
 * 1 = publish telemetry as a schema-v2 envelope over MQTT instead of POSTing
 *     the legacy flat JSON to the dashboard.
 *
 * `wifi_http.cpp` compiles either way, so a node can be reverted to HTTPS by
 * flipping this to 0 and reflashing — no other file changes. That is deliberate:
 * the migration is meant to be revertible one node at a time.
 */
#ifndef AQUA_TRANSPORT_MQTT
#define AQUA_TRANSPORT_MQTT     1
#endif

/* mosquitto on the Pi. See deploy/mosquitto/aqua.conf.
 *
 * Every one of these is overridable with -D at build time, and that is not decoration:
 * a bench node has to point at a developer's broker instead of the station's, and the
 * alternative is editing this file — a tracked file — before every bench session and
 * remembering to revert it afterwards. The client id matters most, because two nodes
 * sharing one is not a warning in the log but a broker-side disconnect loop.
 */
#ifndef AQUA_MQTT_HOST
#define AQUA_MQTT_HOST          "aqua-station.local"
#endif
#ifndef AQUA_MQTT_PORT
#define AQUA_MQTT_PORT          1883
#endif
#ifndef AQUA_MQTT_USER
#define AQUA_MQTT_USER          ""      // blank = anonymous; the LAN config allows it
#endif
#ifndef AQUA_MQTT_PASSWORD
#define AQUA_MQTT_PASSWORD      ""
#endif
#ifndef AQUA_MQTT_KEEPALIVE_S
#define AQUA_MQTT_KEEPALIVE_S   30
#endif
/* Bound for how long the library waits for a broker response. PubSubClient defaults to 15 s
   and busy-waits while it does, so a broker that accepts the connection but never answers
   stalls the reconnect for that long and starves the watchdog. Kept well under the 60 s WDT. */
#ifndef AQUA_MQTT_SOCKET_TIMEOUT_S
#define AQUA_MQTT_SOCKET_TIMEOUT_S 5
#endif
#ifndef AQUA_MQTT_CLIENT_ID
#define AQUA_MQTT_CLIENT_ID     "aqua-sensor-od"
#endif

/*
 * PubSubClient defaults to a 256-byte buffer, which silently truncates any real
 * envelope. Must be >= AQUA_ENVELOPE_MAX_BYTES, because the MQTT payload is the
 * envelope.
 */
#ifndef AQUA_MQTT_BUFFER_BYTES
#define AQUA_MQTT_BUFFER_BYTES  8192
#endif

/* Retry cadence so a missing broker backs off instead of busy-looping. */
#ifndef AQUA_MQTT_RETRY_MS
#define AQUA_MQTT_RETRY_MS      5000UL
#endif

/* ------------------------------ BLE ---------------------------------- */

#define SERVICE_UUID               "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID        "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHARACTERISTIC_NOTIFY_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a9"

#define BLE_ACTIVE_TIME 30000

/* ------------------------------ Timezone ------------------------------ */

#define LOCAL_TZ_OFFSET_SEC (5 * 3600) // UTC-5 (Peru / PET)
#define LOCAL_TZ_NAME       "UTC-5"
