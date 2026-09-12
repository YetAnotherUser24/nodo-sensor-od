# OD-Logger: Aqualabo DIGISENS Optical Dissolved Oxygen Telemetry System

Firmware for the **OD-Logger** telemetry system based on ESP32, dedicated to monitoring water parameters using the **Aqualabo DIGISENS Optical Dissolved Oxygen (OPTOD / OD)** sensor over RS-485 Modbus RTU.

Supports two hardware platforms:
- **Natek-SD v2.0**: Bare ESP32-WROOM-32E MCU + MicroSD + DS3231 RTC + RS-485 + SH1106 OLED + INA226 battery monitor.
- **Natek-GPRS v0.2.0** (LilyGO T-A7670SA): ESP32 + SIM7670 4G/GPRS modem + MicroSD + DS3231 RTC + RS-485 + SH1106 OLED + LiPo ADC.

> 🤖 **Working with AI Agents?** See [AGENTS.md](file:///c:/Users/Sam/Documents/Tesis/Codigos/od-logger/AGENTS.md) for the authoritative architecture, memory rules, and development guide.

---

## Main Features

- 💧 **Optical Dissolved Oxygen (OD) Sensing**
  - Aqualabo DIGISENS OPTOD sensor over Modbus RTU (RS-485 at 9600 baud, 8N1, Slave ID 10).
  - Explicit measurement order trigger (`0x001F` to holding register `0x0001`).
  - Measures 5 float parameters:
    - **Water Temperature**: `sensorReadings[0]` (centi-°C, scale x100)
    - **DO Saturation**: `sensorReadings[1]` (deci-%Sat, scale x10)
    - **Dissolved Oxygen (DO)**: `sensorReadings[2]` (milli-mg/L, scale x1000)
    - **Parameter 3**: `sensorReadings[3]` (centi, scale x100)
    - **Parameter 4**: `sensorReadings[4]` (centi, scale x100)
  - Continuous sampling optimization: Keeps sensor energized between samples to eliminate 1500 ms warmup latency, achieving up to **2-second continuous sampling**.

- 🌐 **Cloud Web Dashboard Control (`AquaControl`)**
  - Real-time HTTPS telemetry streaming to `aquacontrol-two.vercel.app` via `/api/telemetry/sensor`.
  - Bidirectional remote control: `start_experiment`, `stop_experiment`, `start_monitor`, `stop_monitor`, `set_sampling_rate`, `manual_sample`.
  - Detailed protocol guide in [docs/WEB_MASTER_PROTOCOL.md](file:///c:/Users/Sam/Documents/Tesis/Codigos/od-logger/docs/WEB_MASTER_PROTOCOL.md).

- 🔬 **Dedicated Experiment Logging on MicroSD**
  - Records continuous research experiments to RFC 4180 CSV files (`/EXP_01.CSV`, `/EXP_02.CSV`).
  - Flush-on-stop safety to prevent data corruption during unexpected disconnects.

- 📺 **OLED Live Visualization (SH1106 1.3")**
  - Real-time layout showing live DO (mg/L), Saturation (%Sat), Water Temperature (°C), Battery voltage, RTC time, and sampling interval.

- 💡 **Visual LED State Indicators**
  - **3 quick pulses** (100 ms ON / 100 ms OFF x3): Signals entry into **IDLE state** (ready and waiting for commands).
  - **2 pulses** (100 ms ON / 100 ms OFF x2): Signals transition into **Deep Sleep**.

- 🔋 **Battery Management**
  - **Natek-SD**: Monitored via INA226 I2C sensor at address `0x45` (`sensorReadings[5]`).
  - **Natek-GPRS**: Monitored via internal ADC on GPIO 35 (`sensorReadings[5]`).

- 💾 **Resilient Offline Storage**
  - External I2C EEPROM (24LCxxx) ring buffer.
  - MicroSD backup logging daily records to `/YYYYMMDD.CSV`.
  - Store-and-forward telemetry synchronization to ThingsBoard via MQTT over 4G/GPRS.

---

## Build Targets (PlatformIO)

```bash
# 1. Natek-SD v2.0 Web Master (WiFi + SD + INA226 + OLED + AquaControl HTTPS) [ACTIVE TARGET]
pio run -e od-logger-web
pum -e od-logger-web

# 2. Virtual Sensor Benchmark (No hardware sensors required)
pio run -e od-logger-web-virtual

# 3. Natek-SD v2.0 Autonomous Field Target (SD + BLE + OLED + INA226 + Sleep)
pio run -e od-logger-sd
pio run -e od-logger-sd -t upload

# 4. Natek-GPRS v0.2.0 (GPRS + SD + BLE + OLED + Battery ADC + ThingsBoard MQTT)
pio run -e od-logger-gprs
pio run -e od-logger-gprs -t upload

# 5. I2C Diagnostic Scanner
pio run -e i2c-scanner
```

---

## CSV File Formats

### 1. Dedicated Experiment File (`/EXP_XX.CSV`)
```csv
Timestamp,RTC_Sec,DO_mg_L,Sat_Pct,Temp_C,Batt_V
2026-09-12T05:14:00-05:00,842512440,7.850,98.50,25.00,3.95
```

### 2. Daily Continuous Offline Log (`/YYYYMMDD.CSV`)
```csv
datetime, seconds_since_2000, water_temp_centi, do_sat_deci_pct, do_milli_mg_l, param3_centi, param4_centi, battery_mv, rtc_temp_centi, status, sent
2026/09/02 19:40:00, 841779600, 2150, 985, 8456, 120, 0, 3840, 2300, 0, 1
```

---

## Documentation Links
* [AGENTS.md](file:///c:/Users/Sam/Documents/Tesis/Codigos/od-logger/AGENTS.md) — Comprehensive developer & AI agent instruction manual.
* [docs/WEB_MASTER_PROTOCOL.md](file:///c:/Users/Sam/Documents/Tesis/Codigos/od-logger/docs/WEB_MASTER_PROTOCOL.md) — AquaControl REST API & JSON wire protocol.
* [docs/ARCHITECTURE.md](file:///c:/Users/Sam/Documents/Tesis/Codigos/od-logger/docs/ARCHITECTURE.md) — Modular software architecture and storage design.
* [docs/HARDWARE_SETUP.md](file:///c:/Users/Sam/Documents/Tesis/Codigos/od-logger/docs/HARDWARE_SETUP.md) — Wiring and board schematic setup.
