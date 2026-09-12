/**
 * @file sensor_engine.cpp
 * @brief Dedicated sensor engine for Aqualabo DIGISENS Optical Dissolved Oxygen (OD) logger.
 *
 * Implements:
 *   - Aqualabo DIGISENS Modbus RTU driver (RS-485 at 9600 baud 8N1)
 *   - Battery voltage monitoring (ADC on Natek-GPRS, INA226 on Natek-SD)
 *   - Real-time DO sampling for BLE live monitor
 */

#include "sensor_engine.h"
#include "log.h"
#include "modbus_rtu.h"
#include "n_logger_config.h"
#include "oled_display.h"

#include <HardwareSerial.h>
#include <esp32-hal-adc.h>
#include <math.h>

#ifdef SENSOR_INA226
#include <Wire.h>
#endif

/**
 * @brief Safely de-energizes a sensor power/enable pin without turning off the 3.3V I2C OLED rail
 *        if that pin is shared with I2C_EN_PIN.
 */
static inline void safePowerDownSensorPin(int pin, bool activeHigh) {
  if (pin != -1) {
    if (isOledAvailable() && pin == I2C_EN_PIN) {
      return;
    }
    setPinInactive(pin, activeHigh, false);
  }
}

static HardwareSerial odSerial(2);

/* ==================================================================
 *  Aqualabo DIGISENS Modbus RTU Float Decoding & Sanity Checks
 * ================================================================== */

static float decodeFloatFromRegisters(uint16_t firstRegister, uint16_t secondRegister, bool swapWords) {
  union {
    uint32_t u32;
    float f32;
  } value = {};

  if (swapWords) {
    value.u32 = (static_cast<uint32_t>(secondRegister) << 16) | static_cast<uint32_t>(firstRegister);
  } else {
    value.u32 = (static_cast<uint32_t>(firstRegister) << 16) | static_cast<uint32_t>(secondRegister);
  }

  return value.f32;
}

static inline bool isDigisensMeasurementError(uint16_t value) {
  return value == 9999 || value == 9998 || value == 9996;
}

static inline bool isPlausibleMeasurement(float temperature, float od) {
  return !isnan(temperature) && !isnan(od) && !isinf(temperature) && !isinf(od) &&
         temperature >= -40.0f && temperature <= 125.0f && od >= 0.0f && od <= 500.0f;
}

/**
 * @brief Reads all 5 measurements from the DIGISENS OD sensor.
 *
 * Sequence:
 *   1. Modbus write 0x001F -> reg 0x0001 (trigger measurement)
 *   2. Delay sampling delay (reading reg 0x00A4 or fallback 250ms)
 *   3. Modbus read 10 registers from 0x0053
 *   4. Float decoding and scaling to int16_t:
 *        values[0] = Temp (centi °C = °C * 100)
 *        values[1] = DO %Sat (deci %Sat = % * 10)
 *        values[2] = DO (milli-mg/L = mg/L * 1000)
 *        values[3] = ppm (centi = ppm * 100)
 *        values[4] = P4 (* 100)
 *
 * @param[out] values Array of 5 int16_t to receive converted values.
 * @return 0 on success, -2 on comm failure, -5 on Modbus exception, -6 on invalid data.
 */
static int16_t odReadAll(int16_t* values) {
  modbus_rtu::ModbusRtu modbus(odSerial);

  modbus_rtu::ModbusRtu::Config cfg;
  cfg.directionPin = OD_DE_RE_PIN;
  cfg.manualDirection = true;
  cfg.txActiveHigh = true;
  cfg.responseTimeoutMs = 500;
  cfg.txEnableGuardUs = 250;
  cfg.txDisableGuardUs = 250;
  cfg.interFrameGapMs = 4;
  cfg.rxTurnaroundUs = 0;
  modbus.configure(cfg);
  modbus.begin(OD_BAUD, OD_RX_PIN, OD_TX_PIN);

  // Clear stale RX buffer
  modbus.clearRxBuffer();

  // 1. Trigger measurement with retry
  bool triggered = false;
  modbus_rtu::ModbusRtu::WriteResult lastWriteRes;
  for (int retry = 0; retry < 2 && !triggered; retry++) {
    lastWriteRes =
        modbus.writeSingleRegister(OD_SLAVE_ID, OD_MODBUS_REGISTER_ORDER, OD_MODBUS_ORDER_VALUE);
    if (lastWriteRes.success) {
      triggered = true;
    } else {
      if (retry == 0) {
        delay(100);
      }
    }
  }

  #ifdef TEST
  // Just trying out the sensor with random values
  int random = rand() % 10; //random from 0 to 10
  values[0] = static_cast<int16_t>(roundf((25 + (random) )* 100.0f));
  values[1] = static_cast<int16_t>(roundf((75 + (random) ) * 10.0f));
  values[2] = static_cast<int16_t>(roundf((7.5 + (random) )* 1000.0f));
  values[3] = static_cast<int16_t>(roundf((7.5 + (random) )* 100.0f));
  values[4] = static_cast<int16_t>(roundf((5 + (random) )* 100.0f));
  
  return 0;
  #endif
  
  if (!triggered) {
#ifdef DEBUG
    Logf("[OD][ERR] Trigger measurement failed: %s\n", lastWriteRes.error);
#endif
    return lastWriteRes.isException ? -5 : -2;
  }

  // 2. Read sampling delay or use default
  uint32_t samplingDelayMs = OD_DEFAULT_SAMPLING_DELAY_MS;
  auto delayRes = modbus.readRegisters(OD_SLAVE_ID, 0x03, OD_SAMPLING_DELAY_REG, 1);
  if (delayRes.success && delayRes.count >= 1 && delayRes.values[0] > 0) {
    samplingDelayMs = delayRes.values[0];
  }
  delay(samplingDelayMs);

  // 3. Read 10 registers starting at 0x0053
  auto readRes =
      modbus.readRegisters(OD_SLAVE_ID, 0x03, OD_MODBUS_START_ADDR, OD_MODBUS_READ_COUNT);
  if (!readRes.success) {
#ifdef DEBUG
    Logf("[OD][ERR] Read registers failed: %s\n", readRes.error);
#endif
    return readRes.isException ? -5 : -2;
  }

  if (readRes.count < OD_MODBUS_READ_COUNT) {
#ifdef DEBUG
    Logf("[OD][ERR] Incomplete register response: %u/10\n", readRes.count);
#endif
    return -3;
  }

  // 4. Decode 5 floats (normal vs swapped word order)
  // DIGISENS registers: 0x0053=Temp, 0x0055=Param1 (%Sat), 0x0057=Param2 (mg/L), 0x0059=Param3 (ppm), 0x005B=Param4
  float tempNormal = decodeFloatFromRegisters(readRes.values[0], readRes.values[1], false);
  float satNormal  = decodeFloatFromRegisters(readRes.values[2], readRes.values[3], false);
  float doNormal   = decodeFloatFromRegisters(readRes.values[4], readRes.values[5], false);
  float p3Normal   = decodeFloatFromRegisters(readRes.values[6], readRes.values[7], false);
  float p4Normal   = decodeFloatFromRegisters(readRes.values[8], readRes.values[9], false);

  float tempSwapped = decodeFloatFromRegisters(readRes.values[0], readRes.values[1], true);
  float satSwapped  = decodeFloatFromRegisters(readRes.values[2], readRes.values[3], true);
  float doSwapped   = decodeFloatFromRegisters(readRes.values[4], readRes.values[5], true);
  float p3Swapped   = decodeFloatFromRegisters(readRes.values[6], readRes.values[7], true);
  float p4Swapped   = decodeFloatFromRegisters(readRes.values[8], readRes.values[9], true);

  bool useSwapped = false;
  if (!isPlausibleMeasurement(tempNormal, doNormal) && isPlausibleMeasurement(tempSwapped, doSwapped)) {
    useSwapped = true;
  }

  float tempF = useSwapped ? tempSwapped : tempNormal;
  float satF  = useSwapped ? satSwapped : satNormal;
  float doF   = useSwapped ? doSwapped : doNormal;
  float p3F   = useSwapped ? p3Swapped : p3Normal;
  float p4F   = useSwapped ? p4Swapped : p4Normal;

#ifdef DEBUG
  Logf("[OD][DBG] T=%.2f C, Sat=%.2f %%, DO=%.3f mg/L, ppm=%.2f, P4=%.2f (order=%s)\n", tempF, satF,
       doF, p3F, p4F, useSwapped ? "swapped" : "normal");
#endif

  if (isDigisensMeasurementError(static_cast<uint16_t>(tempF)) ||
      isDigisensMeasurementError(static_cast<uint16_t>(doF))) {
#ifdef DEBUG
    Logf("[OD][ERR] DIGISENS error code returned: T=%u, DO=%u\n", static_cast<uint16_t>(tempF),
         static_cast<uint16_t>(doF));
#endif
    return -6;
  }

  if (!isPlausibleMeasurement(tempF, doF)) {
#ifdef DEBUG
    Logf("[OD][ERR] Decoded measurement implausible: T=%.2f, DO=%.3f\n", tempF, doF);
#endif
    return -6;
  }

  values[0] = static_cast<int16_t>(roundf(tempF * 100.0f));  // centi °C
  values[1] = static_cast<int16_t>(roundf(satF * 10.0f));    // deci %Sat (e.g. 98.5% -> 985)
  values[2] = static_cast<int16_t>(roundf(doF * 1000.0f));  // milli-mg/L (e.g. 8.456 mg/L -> 8456)
  values[3] = static_cast<int16_t>(roundf(p3F * 100.0f));    // centi ppm
  values[4] = static_cast<int16_t>(roundf(p4F * 100.0f));

  return 0;
}

/* ==================================================================
 *  Battery Monitoring Drivers (ADC & INA226)
 * ================================================================== */

#ifdef SENSOR_BATTERY_ADC
static uint32_t batteryAdcRead() {
  uint32_t rawMv = analogReadMilliVolts(BATTERY_ADC_PIN);
  return rawMv * BATTERY_ADC_MULTIPLIER;
}
#endif

#ifdef SENSOR_INA226
static void ina226WriteRegister(uint8_t reg, uint16_t value) {
  Wire.beginTransmission(INA226_ADDR);
  Wire.write(reg);
  Wire.write((value >> 8) & 0xFF);
  Wire.write(value & 0xFF);
  Wire.endTransmission();
}

static uint16_t ina226ReadRegister(uint8_t reg) {
  Wire.beginTransmission(INA226_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)INA226_ADDR, (uint8_t)2);
  if (Wire.available() >= 2) {
    uint8_t msb = Wire.read();
    uint8_t lsb = Wire.read();
    return (msb << 8) | lsb;
  }
  return 0;
}

static uint32_t ina226ReadBusVoltage() {
  uint16_t raw = ina226ReadRegister(0x02);
  return (uint32_t)raw * 125 / 100; // raw * 1.25 mV
}

void ina226PowerDown() {
  ina226WriteRegister(0x00, 0x0000);
}
#endif

/* ==================================================================
 *  Main Sensor Sampling Function: sensorReadAll()
 * ================================================================== */

#if defined(SENSOR_VIRTUAL)

void sensorReadAll(TelemetryPayload& tp) {
  static float simAngle = 0.0f;
  simAngle += 0.25f;

  // Realistic simulated aquaculture values:
  // DO: ~7.8 mg/L with slight organic fluctuation (7.0 - 8.6 mg/L)
  // Water Temp: ~24.5 °C (23.8 - 25.2 °C)
  // Oxygen Saturation: ~95% (89 - 101%)
  float simDoMgL = 7.8f + 0.8f * sinf(simAngle);
  float simTempC = 24.5f + 0.7f * cosf(simAngle);
  float simSatPct = 95.0f + 6.0f * sinf(simAngle);

  tp.sensorReadings[0] = (int16_t)roundf(simTempC * 100.0f);   // centi-°C
  tp.sensorReadings[1] = (int16_t)roundf(simSatPct * 10.0f);   // deci-%Sat
  tp.sensorReadings[2] = (int16_t)roundf(simDoMgL * 1000.0f);  // milli-mg/L
  tp.sensorReadings[3] = 0;
  tp.sensorReadings[4] = 0;

  // Battery voltage: healthy 4120 mV default, or real hardware reading if > 3000 mV
#ifdef SENSOR_BATTERY_ADC
  uint32_t realMv = batteryAdcRead();
  tp.battery = (realMv > 3000) ? realMv : 4120;
#elif defined(SENSOR_INA226)
  uint32_t realMv = ina226ReadBusVoltage();
  tp.battery = (realMv > 3000) ? realMv : 4120;
#else
  tp.battery = 4120;
#endif
  tp.sensorReadings[5] = (int16_t)(tp.battery & 0xFFFF);

  for (uint8_t i = 6; i < MAX_SENSOR_READINGS; i++) {
    tp.sensorReadings[i] = 0;
  }

  tp.status = 0x00000000; // nominal, no sensor comm failure

  Logf("[VIRTUAL] DO: %d.%03d mg/L | Temp: %d.%02d C | Sat: %d.%d %% | Bat: %u mV\n",
       tp.sensorReadings[2] / 1000, abs(tp.sensorReadings[2] % 1000),
       tp.sensorReadings[0] / 100, abs(tp.sensorReadings[0] % 100),
       tp.sensorReadings[1] / 10, abs(tp.sensorReadings[1] % 10),
       (unsigned int)tp.battery);
}

int16_t readPrimarySensorRealtime() {
  static float simAngle = 0.0f;
  simAngle += 0.25f;
  float simDoMgL = 7.8f + 0.8f * sinf(simAngle);
  return (int16_t)roundf(simDoMgL * 1000.0f);
}

void stopPrimarySensorRealtime() {
  // No hardware pins to de-energize in virtual mode
}

#else // !SENSOR_VIRTUAL

void sensorReadAll(TelemetryPayload& tp) {
  uint8_t readingIndex = 0;

  /* ---- 1. Optical Dissolved Oxygen (OD) Sensor (Modbus RTU over RS-485) ---- */
#ifdef RS485_EN_PIN
  setPinActive(RS485_EN_PIN, RS485_EN_ACTIVE_HIGH);
#endif
  gpio_hold_dis((gpio_num_t)OD_DE_RE_PIN);

  // Power on
  setPinActive(OD_PWR_PIN, OD_PWR_ACTIVE_HIGH);
  delay(OD_WARMUP_MS);

  // Read samples — reads 10 registers (5 floats)
  uint8_t nread = 0;
  unsigned long sampleStart = millis();
  const unsigned long SAMPLE_TIMEOUT = 5000;

  while (nread < OD_SAMPLE_COUNT && (millis() - sampleStart) < SAMPLE_TIMEOUT) {
    int16_t values[OD_NUM_READINGS] = {0};
    int16_t status = odReadAll(values);

    if (status == 0) {
      for (uint8_t i = 0; i < OD_NUM_READINGS; i++) {
        tp.sensorReadings[readingIndex++] = values[i];
      }
      nread++;
      if (nread < OD_SAMPLE_COUNT) {
        delay(200);
      }
    } else {
      delay(100);
    }
  }

  // Mark failure if no valid sample obtained
  if (nread == 0) {
#ifdef DEBUG
    Logf("[OD] Incomplete: %d/%d samples obtained\n", nread, OD_SAMPLE_COUNT);
#endif
    tp.status = changeBit(tp.status, STATUS_OD_SENSOR_BIT, true);
    for (uint8_t i = 0; i < OD_NUM_READINGS; i++) {
      tp.sensorReadings[readingIndex++] = 0;
    }
  }

  // Power off sensor and bus
  odSerial.end();
  safePowerDownSensorPin(OD_PWR_PIN, OD_PWR_ACTIVE_HIGH);
#ifdef RS485_EN_PIN
  safePowerDownSensorPin(RS485_EN_PIN, RS485_EN_ACTIVE_HIGH);
#endif

  /* ---- 2. Battery Monitoring ---- */
#ifdef SENSOR_BATTERY_ADC
  tp.battery = batteryAdcRead();
  tp.sensorReadings[readingIndex++] = (int16_t)(tp.battery & 0xFFFF);
#elif defined(SENSOR_INA226)
  ina226WriteRegister(0x00, 0x4726); // MODE=110 (bus continuous), AVG=64, VBUSCT=1.1ms
  delay(INA226_WARMUP_MS);
  uint32_t mv = ina226ReadBusVoltage();
#ifdef DEBUG
  Logf("[INA226] Bus voltage: %u mV\n", mv);
#endif
  tp.battery = mv;
  tp.sensorReadings[readingIndex++] = (int16_t)(mv & 0xFFFF);
#endif

  /* Pad remaining slots if any */
  while (readingIndex < MAX_SENSOR_READINGS) {
    tp.sensorReadings[readingIndex++] = 0;
  }
}

/* ==================================================================
 *  Real-Time Sensor Preview Helpers
 * ================================================================== */

static bool realtimeSensorInit = false;

int16_t readPrimarySensorRealtime() {
  if (!realtimeSensorInit) {
#ifdef RS485_EN_PIN
    setPinActive(RS485_EN_PIN, RS485_EN_ACTIVE_HIGH);
#endif
    gpio_hold_dis((gpio_num_t)OD_DE_RE_PIN);
    setPinActive(OD_PWR_PIN, OD_PWR_ACTIVE_HIGH);
    delay(OD_WARMUP_MS);
    realtimeSensorInit = true;
  }

  int16_t values[OD_NUM_READINGS] = {0};
  if (odReadAll(values) == 0) {
    return values[2]; // Return DO in milli-mg/L (e.g. 8456 for 8.456 mg/L)
  }

  return -1;
}

void stopPrimarySensorRealtime() {
  if (!realtimeSensorInit) {
    return;
  }

  odSerial.end();
  safePowerDownSensorPin(OD_PWR_PIN, OD_PWR_ACTIVE_HIGH);
#ifdef RS485_EN_PIN
  safePowerDownSensorPin(RS485_EN_PIN, RS485_EN_ACTIVE_HIGH);
#endif

  realtimeSensorInit = false;
}

#endif // !SENSOR_VIRTUAL

