#pragma once

/*
 * Copy this file to config.h and edit the local copy for deployment values.
 * config.h is ignored by git so secrets stay out of the repository.
 */

/* ----------------------------- SIM / APN ----------------------------- */

#ifndef APN
#define APN "YOUR_APN"
#endif

#ifndef APN_USER
#define APN_USER ""
#endif

#ifndef APN_PASS
#define APN_PASS ""
#endif

#ifndef GSM_PIN
#define GSM_PIN ""
#endif

/* ------------------------- ThingsBoard / MQTT ------------------------- */

#ifndef MQTT_HOST
#define MQTT_HOST "YOUR_MQTT_HOST"
#endif

#ifndef MQTT_PORT
#define MQTT_PORT 1883
#endif

#ifndef MQTT_TOKEN
#define MQTT_TOKEN "YOUR_MQTT_TOKEN"
#endif

#ifndef MQTT_TOPIC
#define MQTT_TOPIC "v1/devices/me/telemetry"
#endif

#ifndef TB_RPC_SUB_TOPIC
#define TB_RPC_SUB_TOPIC "v1/devices/me/rpc/request/+"
#endif

/* ------------------------------ BLE ---------------------------------- */

#ifndef BLE_DEVICE_NAME
#define BLE_DEVICE_NAME "LILYGO_A7670_BLE"
#endif

#ifndef SERVICE_UUID
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#endif

#ifndef CHARACTERISTIC_UUID
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#endif

#ifndef BLE_ACTIVE_TIME
#define BLE_ACTIVE_TIME 30000
#endif

/* ------------------------------- Sampling ----------------------------- */

#ifndef WATER_LEVEL_SENSOR_TYPE
#define WATER_LEVEL_SENSOR_TYPE 1
#endif

#ifndef NUM_READINGS
#define NUM_READINGS 5
#endif

/* ------------------------------ Location ------------------------------ */

#ifndef LOCAL_TZ_OFFSET_SEC
#define LOCAL_TZ_OFFSET_SEC (5 * 3600)
#endif