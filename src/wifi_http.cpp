/**
 * @file wifi_http.cpp
 * @brief WiFi connectivity and HTTPS POST to AquaControl dashboard.
 *
 * Uses ESP32-native WiFi + HTTPClient (built into Arduino framework,
 * zero external dependencies).
 *
 * Guarded by #ifdef WIFI_MODULE.
 */

#ifdef WIFI_MODULE

#include "wifi_http.h"
#include "config.h"
#include "log.h"
#include "n_logger_config.h"
#include "rtc_utils.h"

#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <RtcDS3231.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <esp_sntp.h>
#include <esp_task_wdt.h>
#include <time.h>

extern RtcDS3231<TwoWire> rtc;
extern RtcDateTime rtcNow;
extern char dateStr[20];

/* ----------------------------- WiFi Connect ----------------------------- */

bool wifiConnect() {
  if (WiFi.status() == WL_CONNECTED) {
    return true;
  }

  Logf("WiFi connecting to \"%s\"...", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - t0 > WIFI_CONNECT_TIMEOUT_MS) {
      Logln(" timeout.");
      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);
      return false;
    }
    delay(250);
    Log(".");
  }

  Logf(" connected (IP: %s)\n", WiFi.localIP().toString().c_str());
  syncTimeNtp(3000);

  return true;
}

/* ----------------------------- NTP Time Sync ----------------------------- */

bool syncTimeNtp(uint32_t timeoutMs) {
  if (WiFi.status() != WL_CONNECTED) {
    if (!wifiConnect()) {
      Logln("[NTP] Cannot sync time: WiFi connection failed.");
      return false;
    }
  }

  Logln("[NTP] Synchronizing time via NTP...");
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");

  unsigned long t0 = millis();
  while (sntp_get_sync_status() != SNTP_SYNC_STATUS_COMPLETED && (millis() - t0 < timeoutMs)) {
    delay(100);
    esp_task_wdt_reset();
  }

  time_t utcNow = time(nullptr);
  if (sntp_get_sync_status() == SNTP_SYNC_STATUS_COMPLETED || utcNow >= 1577836800ULL) {
    // Convert authoritative UTC time to local timezone (UTC-5)
    time_t localNow = utcNow - LOCAL_TZ_OFFSET_SEC;
    uint32_t sec2000 = (uint32_t)(localNow - UNIX_OFFSET_2000);
    RtcDateTime ntpDt(sec2000);
    setRtcDateTime(ntpDt);
    formatDateTime(rtcNow);
    Logf("[NTP] Time synchronized successfully: %s %s\n", dateStr, LOCAL_TZ_NAME);

    if (isRtcAvailable()) {
      Logln("[NTP] Hardware DS3231 RTC updated with NTP local time.");
    }
    return true;
  }

  Logln("[NTP] Time sync timed out.");
  return false;
}

#include <lwip/sockets.h>

/* ----------------------------- Persistent HTTP Client ----------------------------- */

static WiFiClientSecure s_client;
static HTTPClient s_http;
static bool s_clientConfigured = false;

static void closeHttpConnection() {
  s_http.end();
  int fd = s_client.fd();
  if (fd >= 0) {
    struct linger sl = { 1, 0 }; // Linger on with timeout 0 -> sends RST, avoids 60s TIME_WAIT
    setsockopt(fd, SOL_SOCKET, SO_LINGER, &sl, sizeof(sl));
  }
  s_client.stop();
}

/* ----------------------------- WiFi Disconnect ----------------------------- */

void wifiDisconnect() {
  closeHttpConnection();

#if !defined(DISABLE_ESPNOW)
  WiFi.disconnect(false, false);
  WiFi.mode(WIFI_STA); // ponytail: preserve STA mode so ESP-NOW direct link does not terminate
  Logln("WiFi disconnected from AP (Radio retained in STA for ESP-NOW).");
#else
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  Logln("WiFi disconnected from AP (Radio powered OFF, ESP-NOW disabled).");
#endif
}

/* ----------------------------- Command Processing & Ack ----------------------------- */

static SensorCommandCallback s_commandCallback = nullptr;

void setSensorCommandCallback(SensorCommandCallback cb) {
  s_commandCallback = cb;
}

static void processTelemetryResponse(const String& response) {
  if (response.indexOf("has_command") == -1 && response.indexOf("command") == -1) {
    return; // Fast-path: neither has_command nor command present
  }

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, response);
  if (err) {
    Logf("[CMD] Response JSON parse error: %s\n", err.c_str());
    return;
  }

  bool hasCommand = doc["has_command"].as<bool>() || 
                    !doc["command"].isNull() || 
                    !doc["pending_command"].isNull();

  if (hasCommand) {
    JsonObject pending = !doc["pending_command"].isNull() 
                           ? doc["pending_command"].as<JsonObject>() 
                           : doc["command"].as<JsonObject>();
    if (pending.isNull()) {
      return;
    }

    const char* cmdId = pending["id"] | pending["command_id"] | "";
    const char* cmdType = pending["command_type"] | "";
    
    JsonObject payload;
    JsonDocument fallbackPayloadDoc;
    if (pending["payload"].is<JsonObject>()) {
      payload = pending["payload"].as<JsonObject>();
    } else if (pending["payload"].is<const char*>()) {
      deserializeJson(fallbackPayloadDoc, pending["payload"].as<const char*>());
      payload = fallbackPayloadDoc.as<JsonObject>();
    }

    const char* action = payload["action"] | "";

    String effectiveAction = action;
    if (effectiveAction.length() == 0) {
      if (strcmp(cmdType, "start") == 0) effectiveAction = "start_monitor";
      else if (strcmp(cmdType, "stop") == 0) effectiveAction = "stop_monitor";
      else effectiveAction = cmdType;
    }

    if (!payload["interval_sec"].is<int>() && effectiveAction == "start_monitor") {
      payload["interval_sec"] = 5;
    }

    String payloadStr;
    serializeJson(payload, payloadStr);

    Logf("[CMD] Received command ID: %s | Action: %s (Type: %s)\n",
         cmdId ? cmdId : "null", effectiveAction.c_str(), cmdType);
    if (s_commandCallback && cmdId && strlen(cmdId) > 0) {
      s_commandCallback(cmdId, effectiveAction.c_str(), payloadStr.c_str());
    }
  }
}

bool acknowledgeSensorCommand(const char* cmdId, bool success, const char* sensorStateJson) {
  if (WiFi.status() != WL_CONNECTED || !cmdId || strlen(cmdId) == 0) {
    return false;
  }

  String ackUrl = String("https://") + AQUACONTROL_HOST + "/api/commands/" + cmdId + "/acknowledge";

  JsonDocument ackDoc;
  ackDoc["success"] = success;
  if (sensorStateJson && strlen(sensorStateJson) > 0) {
    JsonDocument stateDoc;
    if (deserializeJson(stateDoc, sensorStateJson) == DeserializationError::Ok) {
      ackDoc["sensor_state"] = stateDoc.as<JsonObject>();
    }
  }

  String jsonOut;
  serializeJson(ackDoc, jsonOut);

  WiFiClientSecure ackClient;
  ackClient.setInsecure();
  ackClient.setHandshakeTimeout(4);
  ackClient.setTimeout(4);

  HTTPClient ackHttp;
  ackHttp.begin(ackClient, ackUrl);
  ackHttp.addHeader("Content-Type", "application/json");
  ackHttp.addHeader("X-Device-Key", AQUACONTROL_DEVICE_KEY);

  esp_task_wdt_reset();
  int code = ackHttp.POST(jsonOut);
  esp_task_wdt_reset();

  bool ok = (code >= 200 && code < 300);
#ifdef DEBUG
  Logf("[ACK] Command %s -> HTTP %d (%s)\n", cmdId, code, ok ? "OK" : "FAIL");
#endif
  ackHttp.end();
  ackClient.stop();
  return ok;
}

bool pollPendingCommands() {
  if (WiFi.status() != WL_CONNECTED) {
    return false;
  }

  String url = String("https://") + AQUACONTROL_HOST + "/api/commands/pending";

  WiFiClientSecure pollClient;
  pollClient.setInsecure();
  pollClient.setHandshakeTimeout(3);
  pollClient.setTimeout(3);

  HTTPClient pollHttp;
  pollHttp.begin(pollClient, url);
  pollHttp.addHeader("X-Device-Key", AQUACONTROL_DEVICE_KEY);

  esp_task_wdt_reset();
  int httpCode = pollHttp.GET();
  esp_task_wdt_reset();

  if (httpCode > 0) {
    String response = pollHttp.getString();
    pollHttp.end();
    pollClient.stop();
    if (httpCode >= 200 && httpCode < 300) {
      processTelemetryResponse(response);
      return true;
    }
    return false;
  }

  pollHttp.end();
  pollClient.stop();
  return false;
}

/* ----------------------------- HTTPS POST ----------------------------- */

bool publishToAquaControl(uint32_t seconds2000, int16_t* readings,
                          int16_t rtcTempCenti, uint32_t status) {
  if (WiFi.status() != WL_CONNECTED) {
    Logln("publishToAquaControl: WiFi not connected.");
    return false;
  }

  // Build ISO 8601 datetime from seconds since 2000 (local time UTC-5)
  uint64_t unixTs = (uint64_t)seconds2000 + UNIX_OFFSET_2000;
  // If RTC provided an invalid or uninitialized timestamp (< 2020), fall back to NTP system time
  if (unixTs < 1577836800ULL) {
    time_t sysNow = time(nullptr);
    if (sysNow >= 1577836800) {
      unixTs = (uint64_t)(sysNow - LOCAL_TZ_OFFSET_SEC);
      seconds2000 = (uint32_t)(unixTs - UNIX_OFFSET_2000);
    }
  }

  time_t t = (time_t)unixTs;
  struct tm* tm = gmtime(&t);
  char datetime[30];
  snprintf(datetime, sizeof(datetime), "%04d-%02d-%02dT%02d:%02d:%02d-05:00",
           tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday,
           tm->tm_hour, tm->tm_min, tm->tm_sec);


  // Build JSON matching /api/telemetry/sensor expected format
  JsonDocument doc;
  doc["datetime"]          = datetime;
  doc["seconds_since_2000"] = seconds2000;
  doc["water_temp_centi"]  = readings[0];
  doc["do_sat_deci_pct"]   = readings[1];
  doc["do_milli_mg_l"]     = readings[2];
  doc["param3_centi"]      = readings[3];
  doc["param4_centi"]      = readings[4];
#if defined(SENSOR_BATTERY_ADC) || defined(SENSOR_INA226) || defined(SENSOR_VIRTUAL)
  doc["battery_mv"]        = (uint16_t)readings[MAX_SENSOR_READINGS - 1];
#endif

  doc["rtc_temp_centi"]    = rtcTempCenti;
  doc["status"]            = status;
  doc["sent"]              = true;

  String jsonOut;
  serializeJson(doc, jsonOut);

#ifdef DEBUG
  Logln("HTTPS -> AquaControl:");
  Log("   Payload: ");
  Logln(jsonOut.c_str());
#endif

  if (!s_clientConfigured) {
    s_client.setInsecure(); // ponytail: skip cert pin — Vercel rotates certs, pinning would break
    s_client.setHandshakeTimeout(4); // 4s TLS handshake timeout
    s_client.setTimeout(4);          // 4s socket stream timeout
    s_clientConfigured = true;
  }

  String url = String("https://") + AQUACONTROL_HOST + AQUACONTROL_PATH;

  // If not connected, initialize HTTPClient with persistent Keep-Alive
  if (!s_http.connected()) {
    s_http.begin(s_client, url);
    s_http.setReuse(true); // HTTP/1.1 Keep-Alive persistent connection
    s_http.setConnectTimeout(4000);
    s_http.setTimeout(4000);
  }

  s_http.addHeader("Content-Type", "application/json");
  s_http.addHeader("X-Device-Key", AQUACONTROL_DEVICE_KEY);

  esp_task_wdt_reset();
  int httpCode = s_http.POST(jsonOut);
  esp_task_wdt_reset();

  if (httpCode > 0) {
    String response = s_http.getString(); // Consumes body to keep stream aligned for next request
#ifdef DEBUG
    Logf("   HTTP %d: %s\n", httpCode, response.c_str());
#endif
    if (httpCode >= 200 && httpCode < 300) {
      processTelemetryResponse(response);
      return true;
    }
    return false;
  }

  // If POST failed (e.g. idle keep-alive timed out on Vercel), reset socket and retry once
  Logf("   Persistent connection dropped (err %d), reconnecting...\n", httpCode);
  closeHttpConnection();

  s_http.begin(s_client, url);
  s_http.setReuse(true);
  s_http.setConnectTimeout(4000);
  s_http.setTimeout(4000);
  s_http.addHeader("Content-Type", "application/json");
  s_http.addHeader("X-Device-Key", AQUACONTROL_DEVICE_KEY);

  esp_task_wdt_reset();
  httpCode = s_http.POST(jsonOut);
  esp_task_wdt_reset();

  if (httpCode > 0) {
    String response = s_http.getString();
#ifdef DEBUG
    Logf("   HTTP %d (retry): %s\n", httpCode, response.c_str());
#endif
    if (httpCode >= 200 && httpCode < 300) {
      processTelemetryResponse(response);
      return true;
    }
    return false;
  }

  Logf("   HTTP POST retry failed: %s (err %d)\n", s_http.errorToString(httpCode).c_str(), httpCode);
  closeHttpConnection();
  return false;
}

#endif // WIFI_MODULE
