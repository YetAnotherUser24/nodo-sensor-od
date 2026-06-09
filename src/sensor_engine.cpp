/**
 * @file sensor_engine.cpp
 * @brief Sensor engine implementation.
 *
 * Each sensor type is implemented within its own #ifdef block.
 * Adding a new sensor: add a new #ifdef SENSOR_XXX block following
 * the same pattern — define pins/warmup, implement a read function,
 * then hook into sensorReadAll().
 */

#include "sensor_engine.h"
#include "n_logger_config.h"

#include <HardwareSerial.h>
#include <esp32-hal-adc.h>

/* ==================================================================
 *  SENSOR: MaxBotix MB12xx Ultrasonic (UART)
 * ================================================================== */
#ifdef SENSOR_MAXBOTIX_UART

static HardwareSerial maxbotixSerial(2);

/**
 * @brief Reads one distance sample from the MaxBotix sensor.
 * @return Distance in mm, or -2 on timeout/error.
 */
static int16_t maxbotixReadSingle() {
  int16_t result = -2;
  char inData[6] = {0};
  int index = 0;
  unsigned long t0 = millis();
  bool foundR = false;

  // Wait for 'R' start character
  while (!foundR && (millis() - t0) < 500) {
    if (maxbotixSerial.available()) {
      if (maxbotixSerial.read() == 'R') {
        foundR = true;
        break;
      }
    }
  }

  if (!foundR) return -2;

  // Read up to 5 digits after 'R'
  t0 = millis();
  while (index < 5 && (millis() - t0) < 200) {
    if (maxbotixSerial.available()) {
      char c = maxbotixSerial.read();
      if (c >= '0' && c <= '9') {
        inData[index++] = c;
      } else if (c == '\r' || c == '\n') {
        break;
      }
    }
  }

  if (index > 0) {
    inData[index] = '\0';
    result = atoi(inData);
    if (result < 0 || result > 9999) result = -2;
  }

  return result;
}

#endif /* SENSOR_MAXBOTIX_UART */

/* ==================================================================
 *  SENSOR: Water Surface Temperature (I2C)
 * ================================================================== */
#ifdef SENSOR_WATER_TEMP

/**
 * @brief Reads one sample from the water temperature sensor.
 * @return Temperature reading (placeholder returns 0).
 */
static int16_t waterTempReadSingle() {
  // TODO: implement I2C communication with sensor at WATER_TEMP_ADDR
  // Wire library already initialized in main.cpp for RTC/EEPROM
  return 0;
}

#endif /* SENSOR_WATER_TEMP */

/* ==================================================================
 *  SENSOR: Radar distance (placeholder)
 * ================================================================== */
#ifdef SENSOR_RADAR

/**
 * @brief Reads one distance sample from the radar sensor.
 * @return Distance reading (placeholder returns 0).
 */
static int16_t radarReadSingle() {
  // TODO: implement radar sensor read
  return 0;
}

#endif /* SENSOR_RADAR */

/* ==================================================================
 *  SENSOR: Battery voltage via ADC
 * ================================================================== */
#ifdef SENSOR_BATTERY_ADC

/**
 * @brief Reads battery voltage via ADC pin.
 * @return Battery millivolt reading.
 */
static uint16_t batteryAdcRead() {
  analogSetAttenuation(ADC_11db);
  analogReadResolution(12);
  return analogReadMilliVolts(BATTERY_ADC_PIN) * BATTERY_ADC_MULTIPLIER;
}

#endif /* SENSOR_BATTERY_ADC */

/* ==================================================================
 *  sensorReadAll() — Main entry point
 * ================================================================== */

void sensorReadAll(TelemetryPayload& tp) {
  int readingIndex = 0;

  /* ---- MaxBotix UART ---- */
  #ifdef SENSOR_MAXBOTIX_UART
  {
    // Power on
    pinMode(MAXBOTIX_PWR_PIN, OUTPUT);
    digitalWrite(MAXBOTIX_PWR_PIN, HIGH);
    delay(MAXBOTIX_WARMUP_MS);

    // Init UART
    maxbotixSerial.begin(9600, SERIAL_8N1, MAXBOTIX_RX_PIN, MAXBOTIX_TX_PIN);

    // Clear startup noise
    while (maxbotixSerial.available()) maxbotixSerial.read();

    // Read samples
    uint8_t nread = 0;
    unsigned long sampleStart = millis();
    const unsigned long SAMPLE_TIMEOUT = 1500;

    while (nread < MAXBOTIX_NUM_READINGS && (millis() - sampleStart) < SAMPLE_TIMEOUT) {
      int16_t sample = maxbotixReadSingle();
      if (sample > -2) {
        tp.sensorReadings[readingIndex++] = sample;
        nread++;
        if (nread < MAXBOTIX_NUM_READINGS) delay(120);
      } else {
        delay(50);
      }
    }

    // Mark failure if incomplete
    if (nread < MAXBOTIX_NUM_READINGS) {
      tp.status = changeBit(tp.status, STATUS_WATER_LEVEL_SENSOR_BIT, true);
    }

    // Power off
    maxbotixSerial.end();
    digitalWrite(MAXBOTIX_PWR_PIN, LOW);
  }
  #endif /* SENSOR_MAXBOTIX_UART */

  /* ---- Water Temperature ---- */
  #ifdef SENSOR_WATER_TEMP
  {
    // Wire.begin() already called in main setup for RTC/EEPROM
    for (int i = 0; i < WATER_TEMP_NUM_READINGS; i++) {
      tp.sensorReadings[readingIndex++] = waterTempReadSingle();
    }
  }
  #endif /* SENSOR_WATER_TEMP */

  /* ---- Radar ---- */
  #ifdef SENSOR_RADAR
  {
    for (int i = 0; i < RADAR_NUM_READINGS; i++) {
      tp.sensorReadings[readingIndex++] = radarReadSingle();
    }
  }
  #endif /* SENSOR_RADAR */

  /* ---- Battery ADC ---- */
  #ifdef SENSOR_BATTERY_ADC
  {
    tp.battery = batteryAdcRead();
    // Battery reading also maps into sensorReadings for EEPROM/CSV
    tp.sensorReadings[readingIndex++] = (int16_t)(tp.battery & 0xFFFF);
  }
  #endif /* SENSOR_BATTERY_ADC */

  /* Pad remaining slots (if any sensor had fewer readings than expected) */
  while (readingIndex < MAX_SENSOR_READINGS) {
    tp.sensorReadings[readingIndex++] = 0;
  }
}