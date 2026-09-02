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
#include "log.h"
#include "n_logger_config.h"
#include "oled_display.h"

#include <HardwareSerial.h>
#include <esp32-hal-adc.h>

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
    yield();
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
    yield();
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
 *  SENSOR: Water / IR Surface Temperature (MLX90614ESF-DCI over I2C)
 * ================================================================== */
#if defined(SENSOR_WATER_TEMP) || defined(SENSOR_MLX90614)

#include <Wire.h>

/**
 * @brief Reads a temperature RAM register from MLX90614.
 * @param reg RAM register address: 0x07 = Object 1 Temp, 0x06 = Ambient Temp.
 * @return Temperature in decicelsius (0.1°C), or -999 on error.
 */
static int16_t mlx90614ReadTemp(uint8_t reg) {
  Wire.beginTransmission(WATER_TEMP_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) { // repeated start
    return -999;
  }

  if (Wire.requestFrom((uint8_t)WATER_TEMP_ADDR, (uint8_t)3) < 3) {
    return -999;
  }

  uint8_t lsb = Wire.read();
  uint8_t msb = Wire.read();
  uint8_t pec = Wire.read(); // Packet Error Code
  (void)pec;

  uint16_t raw = ((uint16_t)msb << 8) | lsb;

  // Check error bit 7 of MSB or 0 raw
  if (raw == 0 || (msb & 0x80)) {
    return -999;
  }

  // Raw resolution: 0.02 K per LSB
  // Temp in °C x 10 (decicelsius): ((raw * 2) - 27315) / 10
  int32_t tempDeci = ((int32_t)raw * 2 - 27315) / 10;
  return (int16_t)tempDeci;
}

#endif /* SENSOR_WATER_TEMP || SENSOR_MLX90614 */

/* ==================================================================
 *  SENSOR: Radar distance (Modbus RTU over direct UART)
 *
 *  Communicates with the radar over UART (TX/RX only, no RS-485
 *  transceiver). Uses function code 0x03 to read register 0x0001
 * (empty height, in mm) from slave ID 1.
 * ================================================================== */
#ifdef SENSOR_RADAR

#include "modbus_rtu.h"
#include <esp_task_wdt.h>

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

  esp_task_wdt_reset();
  // Read register 0x0001 (empty height) from slave ID 1
  auto result = modbus.readRegisters(RADAR_SLAVE_ID, 0x03, RADAR_MODBUS_REGISTER, 1);

  if (!result.success) {
#ifdef DEBUG
    Logf("[RADAR][ERR] Timeout / no response: %s\n", result.error);
#endif
    return -2; // timeout or comm error
  }

  if (result.isException) {
#ifdef DEBUG
    Logf("[RADAR][ERR] Modbus exception %d: %s\n", result.exceptionCode, result.error);
#endif
    return -5; // Modbus exception
  }

  if (result.count < 1) {
#ifdef DEBUG
    Logf("[RADAR][ERR] Short response: %u register(s) received, expected 1\n", result.count);
#endif
    return -3; // short response
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
static int16_t soil3ReadBoth(int16_t& moisture, int16_t& temperature) {
  modbus_rtu::ModbusRtu modbus(soilSerial);

  modbus_rtu::ModbusRtu::Config cfg;
  cfg.directionPin = SOIL3_DE_RE_PIN;
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
  modbus.begin(SOIL3_BAUD, SOIL3_RX_PIN, SOIL3_TX_PIN);

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
  auto result =
      modbus.readRegisters(SOIL3_SLAVE_ID, 0x03, SOIL3_MODBUS_START_ADDR, SOIL3_MODBUS_READ_COUNT);

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
  auto result =
      modbus.readRegisters(SOIL5_SLAVE_ID, 0x03, SOIL5_MODBUS_START_ADDR, SOIL5_MODBUS_READ_COUNT);

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
 *  SENSOR: Optical Dissolved Oxygen (Aqualabo DIGISENS Modbus RTU)
 *
 *  DIGISENS sensors make measurement only after an order from the master:
 *    1. Write 0x001F to register 0x0001 (start measurement of ALL parameters)
 *    2. Wait sampling delay (read from 0x00A4 or fallback to 250ms)
 *    3. Read 10 holding registers starting at 0x0053:
 *         0x0053: Temperature (°C, float, 2 regs)
 *         0x0055: Parameter 1 (Dissolved Oxygen mg/L or ppm, float, 2 regs)
 *         0x0057: Parameter 2 (DO Saturation %, float, 2 regs)
 *         0x0059: Parameter 3 (float, 2 regs)
 *         0x005B: Parameter 4 (float, 2 regs)
 * ================================================================== */
#ifdef SENSOR_OD

#include "modbus_rtu.h"
#include <math.h>

static HardwareSerial odSerial(2);

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
 * Performs:
 *   1. Modbus write 0x001F -> reg 0x0001 (trigger measurement)
 *   2. Delay sampling delay (reading 0x00A4 or fallback 250ms)
 *   3. Modbus read 10 registers from 0x0053
 *   4. Float decoding and scaling to int16_t:
 *        values[0] = Temp (centi °C = °C * 100)
 *        values[1] = DO (centi mg/L = mg/L * 100)
 *        values[2] = DO %Sat (deci %Sat = % * 10)
 *        values[3] = P3 (* 100)
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
  auto readRes = modbus.readRegisters(OD_SLAVE_ID, 0x03, OD_MODBUS_START_ADDR, OD_MODBUS_READ_COUNT);
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
  float tempNormal = decodeFloatFromRegisters(readRes.values[0], readRes.values[1], false);
  float doNormal = decodeFloatFromRegisters(readRes.values[2], readRes.values[3], false);
  float satNormal = decodeFloatFromRegisters(readRes.values[4], readRes.values[5], false);
  float p3Normal = decodeFloatFromRegisters(readRes.values[6], readRes.values[7], false);
  float p4Normal = decodeFloatFromRegisters(readRes.values[8], readRes.values[9], false);

  float tempSwapped = decodeFloatFromRegisters(readRes.values[0], readRes.values[1], true);
  float doSwapped = decodeFloatFromRegisters(readRes.values[2], readRes.values[3], true);
  float satSwapped = decodeFloatFromRegisters(readRes.values[4], readRes.values[5], true);
  float p3Swapped = decodeFloatFromRegisters(readRes.values[6], readRes.values[7], true);
  float p4Swapped = decodeFloatFromRegisters(readRes.values[8], readRes.values[9], true);

  bool useSwapped = false;
  if (!isPlausibleMeasurement(tempNormal, doNormal) && isPlausibleMeasurement(tempSwapped, doSwapped)) {
    useSwapped = true;
  }

  float tempF = useSwapped ? tempSwapped : tempNormal;
  float doF = useSwapped ? doSwapped : doNormal;
  float satF = useSwapped ? satSwapped : satNormal;
  float p3F = useSwapped ? p3Swapped : p3Normal;
  float p4F = useSwapped ? p4Swapped : p4Normal;

#ifdef DEBUG
  Logf("[OD][DBG] T=%.2f C, DO=%.3f mg/L, Sat=%.2f %%, P3=%.2f, P4=%.2f (order=%s)\n", tempF, doF,
       satF, p3F, p4F, useSwapped ? "swapped" : "normal");
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

  values[0] = static_cast<int16_t>(roundf(tempF * 100.0f)); // centi °C
  values[1] = static_cast<int16_t>(roundf(doF * 100.0f));   // centi mg/L (e.g. 8.45 mg/L -> 845)
  values[2] = static_cast<int16_t>(roundf(satF * 10.0f));   // deci %Sat (e.g. 98.5% -> 985)
  values[3] = static_cast<int16_t>(roundf(p3F * 100.0f));
  values[4] = static_cast<int16_t>(roundf(p4F * 100.0f));

  return 0;
}

#endif /* SENSOR_OD */

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
 *  INA226 I2C voltage monitor helpers
 * ================================================================== */

#ifdef SENSOR_INA226

#include <Wire.h>

/**
 * @brief Writes a 16-bit value to an INA226 register over I2C.
 * @param reg   Register address (0x00 = Config, 0x02 = Bus Voltage, etc.).
 * @param value 16-bit value to write (MSB first per INA226 datasheet).
 */
static void ina226WriteRegister(uint8_t reg, uint16_t value) {
  Wire.beginTransmission(INA226_ADDR);
  Wire.write(reg);
  Wire.write((uint8_t)(value >> 8));   // MSB
  Wire.write((uint8_t)(value & 0xFF)); // LSB
  Wire.endTransmission();
}

/**
 * @brief Reads a 16-bit INA226 register over I2C.
 * @param reg Register address.
 * @return 16-bit register value.
 */
static uint16_t ina226ReadRegister(uint8_t reg) {
  Wire.beginTransmission(INA226_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false); // restart, don't release bus

  uint8_t val = 0;
  if (Wire.requestFrom((uint8_t)INA226_ADDR, (uint8_t)2) >= 2) {
    uint16_t msb = Wire.read();
    uint16_t lsb = Wire.read();
    return (msb << 8) | lsb;
  }
  return 0;
}

/**
 * @brief Reads bus voltage from the INA226 (triggered mode).
 *
 * Call ina226WriteRegister(0x00, 0x3912) first to trigger a conversion,
 * then delay INA226_WARMUP_MS (75ms for 64-avg x 1.1ms + margin),
 * then call this function.
 *
 * @return Bus voltage in millivolts.
 */
static uint16_t ina226ReadBusVoltage() {
  uint16_t raw = ina226ReadRegister(0x02);
  // Bus voltage = raw x 1.25 mV
  // For integer mV: (raw * 125) / 100 = (raw * 5) / 4
  return (uint16_t)(((uint32_t)raw * 5UL) / 4UL);
}

/**
 * @brief Puts the INA226 into power-down mode (MODE bits = 000).
 *
 * Keeps all other config bits (AVG, VBUSCT, VSHCT) unchanged.
 * Called before deep sleep to minimize I2C device quiescent current.
 */
void ina226PowerDown() {
  ina226WriteRegister(0x00, 0x4700);
}

#endif /* SENSOR_INA226 */

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
    digitalWrite(MAXBOTIX_PWR_PIN, LOW); // Force OFF
#ifdef DEBUG
    Logf("[MAXBOTIX][DBG] Cold power cycle: OFF for 100ms\n");
#endif
    delay(100);                           // Discharge period
    digitalWrite(MAXBOTIX_PWR_PIN, HIGH); // Power ON
#ifdef DEBUG
    Logf("[MAXBOTIX][DBG] Power ON, waiting %dms warmup\n", MAXBOTIX_WARMUP_MS);
#endif
    delay(MAXBOTIX_WARMUP_MS); // Stabilization

    // Init UART
    // TX pin set to -1 because MaxBotix is RX-only -- it only transmits data.
    // Driving the TX line could interfere with the sensor's command mode.
    maxbotixSerial.begin(9600, SERIAL_8N1, MAXBOTIX_RX_PIN, -1);

    // Clear startup noise
    while (maxbotixSerial.available())
      maxbotixSerial.read();

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
        if (nread < MAXBOTIX_NUM_READINGS)
          delay(120);
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
    safePowerDownSensorPin(MAXBOTIX_PWR_PIN, MAXBOTIX_PWR_ACTIVE_HIGH);
  }
#endif /* SENSOR_MAXBOTIX_UART */

/* ---- Water / IR Surface Temperature (MLX90614) ---- */
#if defined(SENSOR_WATER_TEMP) || defined(SENSOR_MLX90614)
  {
#if WATER_TEMP_PWR_PIN != -1
    setPinActive(WATER_TEMP_PWR_PIN, WATER_TEMP_PWR_ACTIVE_HIGH); // Power ON
    delay(WATER_TEMP_WARMUP_MS);
#endif

    int16_t objTemp = mlx90614ReadTemp(0x07); // RAM 0x07: Object Temp
    int16_t ambTemp = mlx90614ReadTemp(0x06); // RAM 0x06: Ambient Temp

    if (objTemp > -900 && ambTemp > -900) {
#ifdef DEBUG
      Logf("[MLX90614] Object: %d.%d C, Ambient: %d.%d C\n", objTemp / 10, abs(objTemp % 10),
           ambTemp / 10, abs(ambTemp % 10));
#endif
      tp.sensorReadings[readingIndex++] = objTemp;
      tp.sensorReadings[readingIndex++] = ambTemp;
    } else {
#ifdef DEBUG
      Logf("[MLX90614] Error reading sensor (obj=%d, amb=%d)\n", objTemp, ambTemp);
#endif
      tp.sensorReadings[readingIndex++] = -999;
      tp.sensorReadings[readingIndex++] = -999;
      tp.status = changeBit(tp.status, STATUS_WATER_LEVEL_SENSOR_BIT, true);
    }
  }
#endif /* SENSOR_WATER_TEMP || SENSOR_MLX90614 */

/* ---- Radar (Modbus RTU, direct UART) ---- */
#ifdef SENSOR_RADAR
  {
    // Power on
    setPinActive(RADAR_PWR_PIN, RADAR_PWR_ACTIVE_HIGH);
    esp_task_wdt_reset();
    delay(RADAR_WARMUP_MS);
    esp_task_wdt_reset();

    // Init UART at RADAR_BAUD
    radarSerial.begin(RADAR_BAUD, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);

    // Clear startup noise
    while (radarSerial.available())
      radarSerial.read();

    // Read samples
    uint8_t nread = 0;
    unsigned long sampleStart = millis();

    while (nread < RADAR_NUM_READINGS && (millis() - sampleStart) < RADAR_SAMPLE_TIMEOUT_MS) {
      esp_task_wdt_reset();
      int16_t sample = radarReadSingle();
      if (sample > 0) {
#ifdef DEBUG
        Logf("[RADAR] Sample %d/%d: %d mm\n", nread + 1, RADAR_NUM_READINGS, sample);
#endif
        tp.sensorReadings[readingIndex++] = sample;
        nread++;
        if (nread < RADAR_NUM_READINGS)
          delay(200);
      } else {
#ifdef DEBUG
        Logf("[RADAR] Sample %d/%d: error %d\n", nread + 1, RADAR_NUM_READINGS, sample);
#endif
        delay(100);
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
    setPinInactive(RADAR_PWR_PIN, RADAR_PWR_ACTIVE_HIGH, false);
  }
#endif /* SENSOR_RADAR */

  /* ---- 3-Probe Soil Sensor (Modbus RTU, RS-485) ---- */
#ifdef SENSOR_SOIL_3TIP
  {
#ifdef RS485_EN_PIN
    setPinActive(RS485_EN_PIN, RS485_EN_ACTIVE_HIGH);
#endif
    // Power on
    setPinActive(SOIL3_PWR_PIN, SOIL3_PWR_ACTIVE_HIGH);
    delay(SOIL3_WARMUP_MS);

    // Init UART at 4800 baud
    soilSerial.begin(SOIL3_BAUD, SERIAL_8N1, SOIL3_RX_PIN, SOIL3_TX_PIN);

    // Clear startup noise
    while (soilSerial.available())
      soilSerial.read();

    // Read samples — each sample is a pair (moisture, temperature)
    uint8_t nread = 0;
    unsigned long sampleStart = millis();
    const unsigned long SAMPLE_TIMEOUT = 3000;

    while (nread < SOIL3_SAMPLE_COUNT && (millis() - sampleStart) < SAMPLE_TIMEOUT) {
      int16_t moisture = 0;
      int16_t temperature = 0;
      int16_t status = soil3ReadBoth(moisture, temperature);

      if (status == 0) {
#ifdef DEBUG
        Logf("[SOIL] Sample %d/%d: moisture=%d, temp=%d\n", nread + 1, SOIL3_SAMPLE_COUNT, moisture,
             temperature);
#endif
        tp.sensorReadings[readingIndex++] = moisture;
        tp.sensorReadings[readingIndex++] = temperature;
        nread++;
        if (nread < SOIL3_SAMPLE_COUNT)
          delay(200);
      } else {
#ifdef DEBUG
        Logf("[SOIL] Sample %d/%d: error %d\n", nread + 1, SOIL3_SAMPLE_COUNT, status);
#endif
        delay(100);
      }
    }

    // Mark failure if no samples obtained
    if (nread == 0) {
#ifdef DEBUG
      Logf("[SOIL] Incomplete: %d/%d samples obtained\n", nread, SOIL3_SAMPLE_COUNT);
#endif
      tp.status = changeBit(tp.status, STATUS_SOIL3_SENSOR_BIT, true);
    }

    // Power off
    soilSerial.end();
    safePowerDownSensorPin(SOIL3_PWR_PIN, SOIL3_PWR_ACTIVE_HIGH);
#ifdef RS485_EN_PIN
    safePowerDownSensorPin(RS485_EN_PIN, RS485_EN_ACTIVE_HIGH);
#endif
  }
#endif /* SENSOR_SOIL_3TIP */

  /* ---- 5-Probe Soil Sensor (Modbus RTU, RS-485) ---- */
#ifdef SENSOR_SOIL_5TIP
  {
#ifdef RS485_EN_PIN
    setPinActive(RS485_EN_PIN, RS485_EN_ACTIVE_HIGH);
#endif
    // Power on
    setPinActive(SOIL5_PWR_PIN, SOIL5_PWR_ACTIVE_HIGH);
    delay(SOIL5_WARMUP_MS);

    // Init UART at 4800 baud
    soil5Serial.begin(SOIL5_BAUD, SERIAL_8N1, SOIL5_RX_PIN, SOIL5_TX_PIN);

    // Clear startup noise
    while (soil5Serial.available())
      soil5Serial.read();

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
        if (nread < SOIL5_SAMPLE_COUNT)
          delay(200);
      } else {
        delay(100);
      }
    }

    // Mark failure if no samples obtained
    if (nread == 0) {
      tp.status = changeBit(tp.status, STATUS_SOIL3_SENSOR_BIT, true);
    }

    // Power off
    soil5Serial.end();
    safePowerDownSensorPin(SOIL5_PWR_PIN, SOIL5_PWR_ACTIVE_HIGH);
#ifdef RS485_EN_PIN
    safePowerDownSensorPin(RS485_EN_PIN, RS485_EN_ACTIVE_HIGH);
#endif
  }
#endif /* SENSOR_SOIL_5TIP */

  /* ---- Optical Dissolved Oxygen (OD) Sensor (Modbus RTU, RS-485) ---- */
#ifdef SENSOR_OD
  {
#ifdef RS485_EN_PIN
    setPinActive(RS485_EN_PIN, RS485_EN_ACTIVE_HIGH);
#endif
    // Release GPIO hold from deep sleep on direction pin
    gpio_hold_dis((gpio_num_t)OD_DE_RE_PIN);

    // Power on
    setPinActive(OD_PWR_PIN, OD_PWR_ACTIVE_HIGH);
    delay(OD_WARMUP_MS);

    // Read samples — each sample reads 10 registers (5 floats)
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
        if (nread < OD_SAMPLE_COUNT)
          delay(200);
      } else {
        delay(100);
      }
    }

    // Mark failure if no samples obtained
    if (nread == 0) {
#ifdef DEBUG
      Logf("[OD] Incomplete: %d/%d samples obtained\n", nread, OD_SAMPLE_COUNT);
#endif
      tp.status = changeBit(tp.status, STATUS_OD_SENSOR_BIT, true);
    }

    // Power off
    odSerial.end();
    safePowerDownSensorPin(OD_PWR_PIN, OD_PWR_ACTIVE_HIGH);
#ifdef RS485_EN_PIN
    safePowerDownSensorPin(RS485_EN_PIN, RS485_EN_ACTIVE_HIGH);
#endif
  }
#endif /* SENSOR_OD */

/* ---- Battery ADC ---- */
#ifdef SENSOR_BATTERY_ADC
  {
    tp.battery = batteryAdcRead();
    // Battery reading also maps into sensorReadings for EEPROM/CSV
    tp.sensorReadings[readingIndex++] = (int16_t)(tp.battery & 0xFFFF);
  }
#endif /* SENSOR_BATTERY_ADC */

/* ---- INA226 (I2C voltage monitor, triggered mode) ---- */
#ifdef SENSOR_INA226
  {
    // Trigger bus-voltage conversion (64-avg, 1.1ms each, bus continuous mode)
    ina226WriteRegister(0x00, 0x4726); // MODE=110 (bus continuous), AVG=64, VBUSCT=1.1ms
    delay(INA226_WARMUP_MS);

    uint32_t mv = ina226ReadBusVoltage();
#ifdef DEBUG
    Logf("[INA226] Bus voltage: %u mV\n", mv);
#endif
    tp.battery = mv;
    tp.sensorReadings[readingIndex++] = (int16_t)(mv & 0xFFFF);
  }
#endif /* SENSOR_INA226 */

  /* Pad remaining slots (if any sensor had fewer readings than expected) */
  while (readingIndex < MAX_SENSOR_READINGS) {
    tp.sensorReadings[readingIndex++] = 0;
  }
}

/* ==================================================================
 *  Real-Time Sensor Preview Helpers
 * ================================================================== */

static bool realtimeSensorInit = false;

int16_t readPrimarySensorRealtime() {
  int16_t val = -1;

#ifdef SENSOR_MAXBOTIX_UART
  if (!realtimeSensorInit) {
    setPinActive(MAXBOTIX_PWR_PIN, MAXBOTIX_PWR_ACTIVE_HIGH);
    delay(MAXBOTIX_WARMUP_MS);
    maxbotixSerial.begin(9600, SERIAL_8N1, MAXBOTIX_RX_PIN, -1);
    while (maxbotixSerial.available())
      maxbotixSerial.read();
    realtimeSensorInit = true;
  }
  val = maxbotixReadSingle();
#elif defined(SENSOR_RADAR)
  if (!realtimeSensorInit) {
    setPinActive(RADAR_PWR_PIN, RADAR_PWR_ACTIVE_HIGH);
    delay(RADAR_WARMUP_MS);
    radarSerial.begin(RADAR_BAUD, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);
    realtimeSensorInit = true;
  }
  val = radarReadSingle();
#elif defined(SENSOR_OD)
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
    val = values[1]; // Return DO in centi-mg/L (e.g. 8472 for 84.72 mg/L)
  }
#endif

  return val;
}

void stopPrimarySensorRealtime() {
  if (!realtimeSensorInit)
    return;

#ifdef SENSOR_MAXBOTIX_UART
  maxbotixSerial.end();
  safePowerDownSensorPin(MAXBOTIX_PWR_PIN, MAXBOTIX_PWR_ACTIVE_HIGH);
#elif defined(SENSOR_RADAR)
  radarSerial.end();
  safePowerDownSensorPin(RADAR_PWR_PIN, RADAR_PWR_ACTIVE_HIGH);
#elif defined(SENSOR_OD)
  odSerial.end();
  safePowerDownSensorPin(OD_PWR_PIN, OD_PWR_ACTIVE_HIGH);
#ifdef RS485_EN_PIN
  safePowerDownSensorPin(RS485_EN_PIN, RS485_EN_ACTIVE_HIGH);
#endif
#endif

  realtimeSensorInit = false;
}
