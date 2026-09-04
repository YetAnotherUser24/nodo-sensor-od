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

#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>

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
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  return true;
}

/* ----------------------------- WiFi Disconnect ----------------------------- */

void wifiDisconnect() {
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

/* ----------------------------- HTTPS POST ----------------------------- */

bool publishToAquaControl(uint32_t seconds2000, int16_t* readings,
                          int16_t rtcTempCenti, uint32_t status) {
  if (WiFi.status() != WL_CONNECTED) {
    Logln("publishToAquaControl: WiFi not connected.");
    return false;
  }

  // Build ISO 8601 datetime from seconds since 2000
  uint64_t unixTs = (uint64_t)seconds2000 + UNIX_OFFSET_2000;
  // If RTC provided an invalid or uninitialized timestamp (< 2020), fall back to NTP system time
  if (unixTs < 1577836800ULL) {
    time_t sysNow = time(nullptr);
    if (sysNow >= 1577836800) {
      unixTs = (uint64_t)sysNow;
      seconds2000 = (uint32_t)(unixTs - UNIX_OFFSET_2000);
    }
  }

  time_t t = (time_t)unixTs;
  struct tm* tm = gmtime(&t);
  char datetime[25];
  snprintf(datetime, sizeof(datetime), "%04d-%02d-%02dT%02d:%02d:%02dZ",
           tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday,
           tm->tm_hour, tm->tm_min, tm->tm_sec);


  // Build JSON matching /api/telemetry/sensor expected format
  StaticJsonDocument<384> doc;
  doc["datetime"]          = datetime;
  doc["seconds_since_2000"] = seconds2000;
  doc["water_temp_centi"]  = readings[0];
  doc["do_centi_mg_l"]     = readings[1];
  doc["do_sat_deci_pct"]   = readings[2];
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

  WiFiClientSecure client;
  client.setInsecure(); // ponytail: skip cert pin — Vercel rotates certs, pinning would break

  HTTPClient http;
  String url = String("https://") + AQUACONTROL_HOST + AQUACONTROL_PATH;

  if (!http.begin(client, url)) {
    Logln("   HTTP begin failed");
    return false;
  }

  http.addHeader("Content-Type", "application/json");
  http.addHeader("X-Device-Key", AQUACONTROL_DEVICE_KEY);
  http.setTimeout(10000);

  int httpCode = http.POST(jsonOut);
  String response = http.getString();
  http.end();

#ifdef DEBUG
  Logf("   HTTP %d: %s\n", httpCode, response.c_str());
#endif

  return (httpCode >= 200 && httpCode < 300);
}

#endif // WIFI_MODULE
