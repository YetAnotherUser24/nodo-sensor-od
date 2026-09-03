# OD-Logger: Aqualabo DIGISENS Optical Dissolved Oxygen Telemetry System

Firmware for the **OD-Logger** telemetry system based on ESP32, dedicated to monitoring water parameters using the **Aqualabo DIGISENS Optical Dissolved Oxygen (OPTOD / OD)** sensor over RS-485 Modbus RTU.

Supports two hardware platforms:
- **Natek-GPRS v0.2.0** (LilyGO T-A7670SA): ESP32 + SIM7670 4G/GPRS modem + MicroSD + DS3231 RTC + RS-485 + SH1106 OLED + LiPo ADC.
- **Natek-SD v2.0**: Bare ESP32-WROOM-32E MCU + MicroSD + DS3231 RTC + RS-485 + SH1106 OLED + INA226 battery monitor.

---

## Main Features

- 💧 **Optical Dissolved Oxygen (OD) Sensing**
  - Aqualabo DIGISENS OPTOD sensor over Modbus RTU (RS-485 at 9600 baud, 8N1, Slave ID 10).
  - Explicit measurement order trigger (`0x001F` to holding register `0x0001`).
  - Measures 5 float parameters:
    - **Water Temperature**: `sensorReadings[0]` (centi-°C, scale x100)
    - **Dissolved Oxygen (DO)**: `sensorReadings[1]` (milli-mg/L, scale x1000)
    - **DO Saturation**: `sensorReadings[2]` (deci-%Sat, scale x10)
    - **Parameter 3**: `sensorReadings[3]` (centi, scale x100)
    - **Parameter 4**: `sensorReadings[4]` (centi, scale x100)
  - Sanity validation and Digisens hardware fault detection (codes `9999`, `9998`, `9996`).

- 🔋 **Battery Management**
  - **Natek-GPRS**: Monitored via internal ADC on GPIO 35 (`sensorReadings[5]`).
  - **Natek-SD**: Monitored via INA226 I2C sensor at address `0x45` (`sensorReadings[5]`).

- 📺 **OLED Live Visualization (SH1106 1.3")**
  - Dedicated display layout during `MONITOR` mode showing live DO (mg/L), Saturation (%Sat), Water Temperature (°C), Battery voltage, RTC time, and BLE status.

- 💾 **Resilient Local Storage**
  - External I2C EEPROM (24LCxxx) ring buffer.
  - MicroSD backup logging daily records to `/YYYYMMDD.CSV`.
  - Store-and-forward telemetry synchronization to ThingsBoard via MQTT over 4G/GPRS.

- 📶 **Bluetooth Low Energy (BLE)**
  - Configuration of measurement / BLE / GPRS intervals (`T-<m>-<b>-<g>`).
  - Real-time continuous sampling mode (`MONITOR`) streaming live DO measurements over GATT notifications.
  - Time synchronization (`D-YYYY-MM-DD-HH-MM-SS`).

- ⏱️ **Power Management**
  - Deep-sleep power management with GPIO hold isolation on RS-485 direction and power rails.

---

## Build Targets (PlatformIO)

Build and upload using PlatformIO:

```bash
# 1. Natek-GPRS v0.2.0 (GPRS + SD + BLE + OLED + Battery ADC + OD Sensor)
pio run -e od-logger-gprs
pio run -e od-logger-gprs -t upload

# 2. Natek-SD v2.0 (SD + BLE + OLED + INA226 + OD Sensor)
pio run -e od-logger-sd
pio run -e od-logger-sd -t upload
```

---

## Telemetry Format (ThingsBoard MQTT)

```json
{
  "ts": 1732837080000,
  "values": {
    "battery": 3840,
    "temperature": 2300,
    "status": 0,
    "sensorReadings": [2150, 8456, 985, 120, 0, 3840]
  }
}
```

* `sensorReadings`:
  - `[0]`: Water Temp (21.50 °C)
  - `[1]`: Dissolved Oxygen (8.456 mg/L)
  - `[2]`: DO Saturation (98.5 %Sat)
  - `[3]`: Parameter 3 (1.20)
  - `[4]`: Parameter 4 (0.00)
  - `[5]`: Battery (3840 mV)
* `temperature`: DS3231 RTC internal temperature (23.00 °C).
* `status`: 32-bit status bitmask (`STATUS_OD_SENSOR_BIT = 22`).

---

## CSV File Format (`/YYYYMMDD.CSV`)

```csv
datetime, seconds_since_2000, water_temp_centi, do_milli_mg_l, do_sat_deci_pct, param3_centi, param4_centi, battery_mv, rtc_temp_centi, status, sent
2026/09/02 19:40:00, 841779600, 2150, 8456, 985, 120, 0, 3840, 2300, 0, 1
```
