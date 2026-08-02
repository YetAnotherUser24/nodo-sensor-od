/**
 * @file sensor_engine.cpp
 * @brief Sensor engine implementation.
 *
 * Each sensor type is implemented within its own #ifdef block.
 * Adding a new sensor: add a new #ifdef SENSOR_XXX block following
 * the same pattern — define pins/warmup, implement a read function,
 * then hook into sensorReadAll().
 *
 * NOTE: SENSOR_RADAR and SENSOR_SOIL_3TIP (3-probe soil sensor) use the same UART (Serial2)
 * and power pin (GPIO 19) and are mutually exclusive at build time.
 */

#include "sensor_engine.h"
#include "n_logger_config.h"
#include "log.h"

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

  // Flush stale bytes and report
  uint8_t dropped = 0;
  while (maxbotixSerial.available()) {
    maxbotixSerial.read();
    dropped++;
  }
#ifdef DEBUG
  if (dropped > 0) {
    Logf("[MAXBOTIX][DBG] Flushed %u stale byte(s)\n", dropped);
  }
#endif

  // Wait for 'R' start character
  while (!foundR && (millis() - t0) < 500) {
    if (maxbotixSerial.available()) {
      if (maxbotixSerial.read() == 'R') {
        foundR = true;
        break;
      }
    }
  }

  if (!foundR) {
#ifdef DEBUG
    Logf("[MAXBOTIX][ERR] Start char 'R' not found\n");
#endif
    return -2;
  }

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
#ifdef DEBUG
    Logf("[MAXBOTIX][DBG] Raw ASCII: \"%s\"\n", inData);
#endif
    result = atoi(inData);
    if (result < 0 || result > 9999) {
#ifdef DEBUG
      Logf("[MAXBOTIX][ERR] Invalid value: %d (out of range)\n", result);
#endif
      result = -2;
    } else {
#ifdef DEBUG
      Logf("[MAXBOTIX][DBG] Parsed value: %d mm\n", result);
#endif
    }
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
 *  SENSOR: Radar distance (Modbus RTU over direct UART)
 *
 *  Communicates with the radar over UART (TX/RX only, no RS-485
 *  transceiver). Uses function code 0x03 to read register 0x0001
 * (empty height, in mm) from slave ID 1.
 * ================================================================== */
#ifdef SENSOR_RADAR

#include "modbus_rtu.h"

static HardwareSerial radarSerial(2);

/**
 * @brief Reads one distance sample from the radar sensor.
 * @return Distance in mm, or -2 on timeout, -3 on short response,
 *         -4 on CRC mismatch, -5 on Modbus exception.
 */
static int16_t radarReadSingle() {
  // Create a ModbusRtu instance on radarSerial (direct UART, no direction pin).
  // manualDirection defaults to false, so no RS-485 DE/RE control is used.
  modbus_rtu::ModbusRtu modbus(radarSerial);

  modbus_rtu::ModbusRtu::Config cfg;
  cfg.responseTimeoutMs = 500;
  cfg.interFrameGapMs = 4;
  modbus.configure(cfg);
  // Radar uses direct UART (no RS-485 transceiver), so modbus.begin() is NOT needed.
  // Direction pin control is handled by the ModbusRtu class internally only when
  // manualDirection is true; for direct UART it defaults to false.

  // Flush stale bytes and report
  uint8_t dropped = 0;
  while (radarSerial.available()) {
    radarSerial.read();
    dropped++;
  }
#ifdef DEBUG
  if (dropped > 0) {
    Logf("[RADAR][DBG] Flushed %u stale byte(s)\n", dropped);
  }
#endif

  // Read register 0x0001 (empty height) from slave ID 1
  auto result = modbus.readRegisters(RADAR_SLAVE_ID, 0x03, RADAR_MODBUS_REGISTER, 1);

  if (!result.success) {
#ifdef DEBUG
    Logf("[RADAR][ERR] Timeout / no response: %s\n", result.error);
#endif
    return -2;  // timeout or comm error
  }

  if (result.isException) {
#ifdef DEBUG
    Logf("[RADAR][ERR] Modbus exception %d: %s\n", result.exceptionCode, result.error);
#endif
    return -5;  // Modbus exception
  }

  if (result.count < 1) {
#ifdef DEBUG
    Logf("[RADAR][ERR] Short response: %u register(s) received, expected 1\n", result.count);
#endif
    return -3;  // short response
  }

  int16_t val = static_cast<int16_t>(result.values[0]);

#ifdef DEBUG
  if (val <= 0) {
    Logf("[RADAR][ERR] Invalid value: %d\n", val);
  } else {
    Logf("[RADAR][DBG] Value: %d mm\n", val);
  }
#endif

  return val;
}

#endif /* SENSOR_RADAR */

/* ==================================================================
 *  SENSOR: 3-Probe Soil Moisture/Temperature (Modbus RTU over RS-485)
 *
 *  Reads 2 holding registers starting at 0x0000 from slave ID 1:
 *    register 0x0000 = moisture (%RH, ÷10)
 *    register 0x0001 = temperature (°C, ÷10, signed)
 *  RS-485 transceiver direction control on D0 (active high).
 * ================================================================== */
#ifdef SENSOR_SOIL_3TIP

#include "modbus_rtu.h"

static HardwareSerial soilSerial(2);

/**
 * @brief Reads both moisture and temperature from the soil sensor.
 *
 * Performs one Modbus read of 2 registers starting at 0x0000.
 * Stores moisture in the provided reference and temperature in the other.
 *
 * @param[out] moisture   Raw moisture value (÷10 = %RH)
 * @param[out] temperature Raw temperature value (÷10 = °C, signed)
 * @return 0 on success, -2 on comm failure, -5 on Modbus exception.
 */
static int16_t soilReadBoth(int16_t& moisture, int16_t& temperature) {
  modbus_rtu::ModbusRtu modbus(soilSerial);

  modbus_rtu::ModbusRtu::Config cfg;
  cfg.directionPin = SOIL_DE_RE_PIN;
  cfg.manualDirection = true;
  cfg.txActiveHigh = true;
  cfg.responseTimeoutMs = 500;
  cfg.txEnableGuardUs = 250;
  cfg.txDisableGuardUs = 250;
  cfg.interFrameGapMs = 4;
  cfg.rxTurnaroundUs = 0;
  modbus.configure(cfg);
  // CRITICAL: modbus.begin() initializes the DE/RE direction pin for RS-485.
  // Without this call, the transceiver never enables its receiver, making
  // communication impossible even though the UART lines are correct.
  modbus.begin(SOIL_BAUD, SOIL_RX_PIN, SOIL_TX_PIN);

  // Flush stale bytes and report
  uint8_t dropped = 0;
  while (soilSerial.available()) {
    soilSerial.read();
    dropped++;
  }
#ifdef DEBUG
  if (dropped > 0) {
    Logf("[SOIL][DBG] Flushed %u stale byte(s)\n", dropped);
  }
#endif

  // Read 2 registers starting at 0x0000 (moisture + temperature)
  auto result = modbus.readRegisters(SOIL_SLAVE_ID, 0x03, SOIL_MODBUS_START_ADDR, SOIL_MODBUS_READ_COUNT);

  if (!result.success) {
#ifdef DEBUG
    Logf("[SOIL][ERR] Timeout / no response: %s\n", result.error);
#endif
    return -2;
  }

  if (result.isException) {
#ifdef DEBUG
    Logf("[SOIL][ERR] Modbus exception %d: %s\n", result.exceptionCode, result.error);
#endif
    return -5;
  }

  if (result.count < 2) {
#ifdef DEBUG
    Logf("[SOIL][ERR] Short response: %u register(s) received, expected 2\n", result.count);
#endif
    return -3;
  }

  moisture = static_cast<int16_t>(result.values[0]);
  temperature = static_cast<int16_t>(result.values[1]);

#ifdef DEBUG
  Logf("[SOIL][DBG] Moisture=%d Temp=%d\n", moisture, temperature);
#endif

  return 0;
}

#endif /* SENSOR_SOIL_3TIP */

/* ==================================================================
 *  SENSOR: 5-Probe Soil Comprehensive (Modbus RTU over RS-485)
 *
 *  Reads 7 holding registers starting at 0x0000 from slave ID 1:
 *    register 0x0000 = moisture (%RH, ÷10)
 *    register 0x0001 = temperature (°C, ÷10, signed)
 *    register 0x0002 = conductivity (EC, µS/cm, direct)
 *    register 0x0003 = pH (÷10)
 *    register 0x0004 = nitrogen (N, mg/kg, direct)
 *    register 0x0005 = phosphorus (P, mg/kg, direct)
 *    register 0x0006 = potassium (K, mg/kg, direct)
 *  RS-485 transceiver direction control on D0 (active high).
 * ================================================================== */
#ifdef SENSOR_SOIL_5TIP

#include "modbus_rtu.h"

static HardwareSerial soil5Serial(2);

/**
 * @brief Reads all 7 measurements from the 5-probe soil sensor.
 *
 * Performs one Modbus read of 7 registers starting at 0x0000.
 * Fills the provided array with raw values in this order:
 *   moisture, temperature, conductivity, pH, nitrogen, phosphorus, potassium
 *
 * @param[out] values Array of 7 int16_t to receive raw register values.
 * @return 0 on success, -2 on comm failure, -5 on Modbus exception.
 */
static int16_t soil5ReadAll(int16_t* values) {
  modbus_rtu::ModbusRtu modbus(soil5Serial);

  modbus_rtu::ModbusRtu::Config cfg;
  cfg.directionPin = SOIL5_DE_RE_PIN;
  cfg.manualDirection = true;
  cfg.txActiveHigh = true;
  cfg.responseTimeoutMs = 500;
  cfg.txEnableGuardUs = 250;
  cfg.txDisableGuardUs = 250;
  cfg.interFrameGapMs = 6;
  cfg.rxTurnaroundUs = 0;
  modbus.configure(cfg);
  // CRITICAL: modbus.begin() initializes the DE/RE direction pin for RS-485.
  // Without this call, the transceiver never enables its receiver, making
  // communication impossible even though the UART lines are correct.
  modbus.begin(SOIL5_BAUD, SOIL5_RX_PIN, SOIL5_TX_PIN);

  // Read 7 registers starting at 0x0000
  auto result = modbus.readRegisters(SOIL5_SLAVE_ID, 0x03, SOIL5_MODBUS_START_ADDR, SOIL5_MODBUS_READ_COUNT);

  if (!result.success) {
    return -2;
  }

  if (result.isException) {
    return -5;
  }

  if (result.count < SOIL5_MODBUS_READ_COUNT) {
    return -3;
  }

  for (uint8_t i = 0; i < SOIL5_MODBUS_READ_COUNT; i++) {
    values[i] = static_cast<int16_t>(result.values[i]);
  }

  return 0;
}

#endif /* SENSOR_SOIL_5TIP */

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
    // Power on — cold-start cycle to ensure clean sensor reset after deep sleep
    pinMode(MAXBOTIX_PWR_PIN, OUTPUT);
    gpio_hold_dis((gpio_num_t)MAXBOTIX_PWR_PIN);
    digitalWrite(MAXBOTIX_PWR_PIN, LOW);   // Force OFF
#ifdef DEBUG
    Logf("[MAXBOTIX][DBG] Cold power cycle: OFF for 100ms\n");
#endif
    delay(100);                             // Discharge period
    digitalWrite(MAXBOTIX_PWR_PIN, HIGH);  // Power ON
#ifdef DEBUG
    Logf("[MAXBOTIX][DBG] Power ON, waiting %dms warmup\n", MAXBOTIX_WARMUP_MS);
#endif
    delay(MAXBOTIX_WARMUP_MS);             // Stabilization

    // Init UART
    // TX pin set to -1 because MaxBotix is RX-only -- it only transmits data.
    // Driving the TX line could interfere with the sensor's command mode.
    maxbotixSerial.begin(9600, SERIAL_8N1, MAXBOTIX_RX_PIN, -1);

    // Clear startup noise
    while (maxbotixSerial.available()) maxbotixSerial.read();

    // Read samples
    uint8_t nread = 0;
    unsigned long sampleStart = millis();
    const unsigned long SAMPLE_TIMEOUT = 1500;

    while (nread < MAXBOTIX_NUM_READINGS && (millis() - sampleStart) < SAMPLE_TIMEOUT) {
      int16_t sample = maxbotixReadSingle();
      if (sample > -2) {
#ifdef DEBUG
        Logf("[MAXBOTIX] Sample %d/%d: %d mm\n", nread + 1, MAXBOTIX_NUM_READINGS, sample);
#endif
        tp.sensorReadings[readingIndex++] = sample;
        nread++;
        if (nread < MAXBOTIX_NUM_READINGS) delay(120);
      } else {
#ifdef DEBUG
        Logf("[MAXBOTIX] Sample %d/%d: error %d\n", nread + 1, MAXBOTIX_NUM_READINGS, sample);
#endif
        delay(50);
      }
    }

    // Mark failure if incomplete
    if (nread < MAXBOTIX_NUM_READINGS) {
#ifdef DEBUG
      Logf("[MAXBOTIX] Incomplete: %d/%d samples obtained\n", nread, MAXBOTIX_NUM_READINGS);
#endif
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

  /* ---- Radar (Modbus RTU, direct UART) ---- */
  #ifdef SENSOR_RADAR
  {
    // Power on
    pinMode(RADAR_PWR_PIN, OUTPUT);
    digitalWrite(RADAR_PWR_PIN, HIGH);
    delay(RADAR_WARMUP_MS);

    // Init UART at 9600 baud
    radarSerial.begin(RADAR_BAUD, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);

    // Clear startup noise
    while (radarSerial.available()) radarSerial.read();

    // Read samples
    uint8_t nread = 0;
    unsigned long sampleStart = millis();

    while (nread < RADAR_NUM_READINGS && (millis() - sampleStart) < RADAR_SAMPLE_TIMEOUT_MS) {
      int16_t sample = radarReadSingle();
      if (sample > 0) {
#ifdef DEBUG
        Logf("[RADAR] Sample %d/%d: %d mm\n", nread + 1, RADAR_NUM_READINGS, sample);
#endif
        tp.sensorReadings[readingIndex++] = sample;
        nread++;
        if (nread < RADAR_NUM_READINGS) delay(200);
      } else {
#ifdef DEBUG
        Logf("[RADAR] Sample %d/%d: error %d\n", nread + 1, RADAR_NUM_READINGS, sample);
#endif
        delay(50);
      }
    }

    // Mark failure if incomplete
    if (nread < RADAR_NUM_READINGS) {
#ifdef DEBUG
      Logf("[RADAR] Incomplete: %d/%d samples obtained\n", nread, RADAR_NUM_READINGS);
#endif
      tp.status = changeBit(tp.status, STATUS_RADAR_SENSOR_BIT, true);
    }

    // Power off
    radarSerial.end();
    digitalWrite(RADAR_PWR_PIN, LOW);
  }
  #endif /* SENSOR_RADAR */

/* ---- 3-Probe Soil Sensor (Modbus RTU, RS-485) ---- */
  #ifdef SENSOR_SOIL_3TIP
  {
    // Enable sensor (if EN pin configured)
    #if SOIL_EN_PIN != -1
    pinMode(SOIL_EN_PIN, OUTPUT);
    digitalWrite(SOIL_EN_PIN, SOIL_EN_ACTIVE_HIGH ? HIGH : LOW);
    delay(10);
    #endif

    // Power on
    pinMode(SOIL_PWR_PIN, OUTPUT);
    digitalWrite(SOIL_PWR_PIN, HIGH);
    delay(SOIL_WARMUP_MS);

    // Init UART at 4800 baud
    soilSerial.begin(SOIL_BAUD, SERIAL_8N1, SOIL_RX_PIN, SOIL_TX_PIN);

    // Clear startup noise
    while (soilSerial.available()) soilSerial.read();

    // Read samples — each sample is a pair (moisture, temperature)
    uint8_t nread = 0;
    unsigned long sampleStart = millis();
    const unsigned long SAMPLE_TIMEOUT = 3000;

    while (nread < SOIL_SAMPLE_COUNT && (millis() - sampleStart) < SAMPLE_TIMEOUT) {
      int16_t moisture = 0;
      int16_t temperature = 0;
      int16_t status = soilReadBoth(moisture, temperature);

      if (status == 0) {
#ifdef DEBUG
        Logf("[SOIL] Sample %d/%d: moisture=%d, temp=%d\n", nread + 1, SOIL_SAMPLE_COUNT, moisture, temperature);
#endif
        tp.sensorReadings[readingIndex++] = moisture;
        tp.sensorReadings[readingIndex++] = temperature;
        nread++;
        if (nread < SOIL_SAMPLE_COUNT) delay(200);
      } else {
#ifdef DEBUG
        Logf("[SOIL] Sample %d/%d: error %d\n", nread + 1, SOIL_SAMPLE_COUNT, status);
#endif
        delay(100);
      }
    }

    // Mark failure if no samples obtained
    if (nread == 0) {
#ifdef DEBUG
      Logf("[SOIL] Incomplete: %d/%d samples obtained\n", nread, SOIL_SAMPLE_COUNT);
#endif
      tp.status = changeBit(tp.status, STATUS_SOIL_SENSOR_BIT, true);
    }

    // Power off
    soilSerial.end();
    digitalWrite(SOIL_PWR_PIN, LOW);

    // Disable sensor (if EN pin configured)
    #if SOIL_EN_PIN != -1
    digitalWrite(SOIL_EN_PIN, SOIL_EN_ACTIVE_HIGH ? LOW : HIGH);
    #endif
  }
  #endif /* SENSOR_SOIL_3TIP */

/* ---- 5-Probe Soil Sensor (Modbus RTU, RS-485) ---- */
  #ifdef SENSOR_SOIL_5TIP
  {
    // Enable sensor (if EN pin configured)
    #if SOIL5_EN_PIN != -1
    pinMode(SOIL5_EN_PIN, OUTPUT);
    digitalWrite(SOIL5_EN_PIN, SOIL5_EN_ACTIVE_HIGH ? HIGH : LOW);
    delay(10);
    #endif

    // Power on
    pinMode(SOIL5_PWR_PIN, OUTPUT);
    digitalWrite(SOIL5_PWR_PIN, HIGH);
    delay(SOIL5_WARMUP_MS);

    // Init UART at 4800 baud
    soil5Serial.begin(SOIL5_BAUD, SERIAL_8N1, SOIL5_RX_PIN, SOIL5_TX_PIN);

    // Clear startup noise
    while (soil5Serial.available()) soil5Serial.read();

    // Read samples — each sample reads all 7 registers
    uint8_t nread = 0;
    unsigned long sampleStart = millis();
    const unsigned long SAMPLE_TIMEOUT = 3000;

    while (nread < SOIL5_SAMPLE_COUNT && (millis() - sampleStart) < SAMPLE_TIMEOUT) {
      int16_t values[SOIL5_MODBUS_READ_COUNT] = {0};
      int16_t status = soil5ReadAll(values);

      if (status == 0) {
        for (uint8_t i = 0; i < SOIL5_MODBUS_READ_COUNT; i++) {
          tp.sensorReadings[readingIndex++] = values[i];
        }
        nread++;
        if (nread < SOIL5_SAMPLE_COUNT) delay(200);
      } else {
        delay(100);
      }
    }

    // Mark failure if no samples obtained
    if (nread == 0) {
      tp.status = changeBit(tp.status, STATUS_SOIL_SENSOR_BIT, true);
    }

    // Power off
    soil5Serial.end();
    digitalWrite(SOIL5_PWR_PIN, LOW);

    // Disable sensor (if EN pin configured)
    #if SOIL5_EN_PIN != -1
    digitalWrite(SOIL5_EN_PIN, SOIL5_EN_ACTIVE_HIGH ? LOW : HIGH);
    #endif
  }
  #endif /* SENSOR_SOIL_5TIP */

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