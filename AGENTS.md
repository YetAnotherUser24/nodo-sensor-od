# AGENTS.md — Developer & AI Agent Guide for OD-Logger

Welcome to **OD-Logger**. This document is the authoritative specification and architectural guide for AI coding agents and developers working on this codebase.

---

## 1. Project Overview & Mission

**OD-Logger** is an autonomous, industrial-grade telemetry firmware written in C++ (Arduino/ESP-IDF framework under PlatformIO) targeting ESP32 microcontrollers.

Its primary purpose is real-time and periodic monitoring of water quality parameters using the **Aqualabo DIGISENS Optical Dissolved Oxygen (OPTOD)** sensor over RS-485 Modbus RTU, with dual operational modes:
1. **Cloud Web Dashboard (`AquaControl`)**: Real-time 24/7 telemetry streaming and bidirectional remote control via HTTPS REST over WiFi.
2. **Offline Field Logging**: Periodic sampling and direct daily CSV logging to MicroSD (`/YYYYMMDD.CSV`), with deep-sleep power cycling.

---

## 2. Hardware Platforms

The codebase strictly abstracts hardware board variations via [src/board.h](file:///c:/Users/Sam/Documents/Tesis/Codigos/od-logger/src/board.h).

| Platform | Build Flag | Target MCU | Key Peripherals | Power / Sensor Pins |
| :--- | :--- | :--- | :--- | :--- |
| **Natek-SD v2.0** | `-DBOARD_NATEK_SD_V2` | ESP32-WROOM-32E (bare) | INA226 (`0x45`), 1.3" SH1106 OLED (`0x3C`), DS3231 RTC (`0x68`), MicroSD (SPI), RS-485 | `EN_5V=19`, `OD_PWR=12`, `RS485_EN=27`, `I2C_EN=25`, `LED_PIN=32` |
| **LilyGO T3 LoRa32** | `-DBOARD_LILYGO_LORA32` | LilyGO T3 LoRa32 v1.6 | SX1278 LoRa, onboard 0.96" SSD1306 OLED (`0x3C`), DS3231 RTC, MicroSD, RS-485 | `LORA_CS=18`, `LORA_RST=23`, `LORA_DIO0=26`, `OD_PWR=25`, `LED_PIN=25` |

### Critical Electrical & Power Rail Rules
* **3.3V I2C Rail (`I2C_EN_PIN = 25`)**: Powers the DS3231 RTC and SH1106 OLED (Natek-SD). **Never de-energize** this pin during continuous monitoring or active experiment loops, as resetting OLED power crashes the I2C bus.
* **OLED Controller Differences**:
  - **1.3" SH1106**: Used on Natek-SD. Requires 2-pixel column offset (`0x02`) and internal DC-DC commands (`0xAD 0x8B`, `0x33`).
  - **0.96" SSD1306**: Onboard display for LilyGO T3 LoRa32. Requires 0-pixel column offset (`0x00`), charge pump enable (`0x8D 0x14`), and page addressing mode (`0x20 0x02`).
* **Dual SPI Controllers (LilyGO T3 LoRa32)**:
  - **VSPI (`SPI3`)**: Dedicated to LoRa SX1278 (`SCK=5`, `MISO=19`, `MOSI=27`, `CS=18`, `RST=23`, `DIO0=26`). Initialized via default `SPI` object or `loraSpiInit()`.
  - **HSPI (`SPI2`)**: Dedicated to MicroSD card slot (`SCK=14`, `MISO=2`, `MOSI=15`, `CS=13`). Driven independently by `sdSPI`.
  - Both buses have zero shared pins, preventing CS contention and allowing simultaneous transceiver and logging transactions.
* **5V Boost Rail (`EN_5V_PIN = 19`) & Sensor Power (`OD_PWR_PIN = 12`)**: Supplies the MAX13487 RS-485 transceiver and the OPTOD sensor.
* **Sensor Warmup Optimization**: The OPTOD sensor requires ~1500 ms to boot from a cold power-on. During continuous monitoring or experiments (`experimentActive || webMonitorActive`), the sensor rail is **kept energized** between samples, enabling up to **2-second continuous sampling rates**. Call `stopPrimarySensorRealtime()` upon stopping to cut power.

---

## 3. PlatformIO Environments Matrix

Inspect [platformio.ini](file:///c:/Users/Sam/Documents/Tesis/Codigos/od-logger/platformio.ini) before running builds:

| Environment | Board | Primary Modules | Description |
| :--- | :--- | :--- | :--- |
| `od-logger-web` | Natek-SD v2.0 | WiFi + SD + INA226 + OLED + Web Master | **Current active test target**. Connects to WiFi, syncs time via NTP, logs experiments to SD, streams to AquaControl via HTTPS. |
| `od-logger-web-virtual`| Natek-SD v2.0 | WiFi + Virtual Sensor + Web Master | Bench simulation target without physical OD or INA226 sensors. Generates synthetic telemetry for UI testing. |
| `od-logger-sd` | Natek-SD v2.0 | SD + BLE + INA226 + OLED + Sleep | Autonomous low-power field logger. Samples periodically, directly appends to SD `/YYYYMMDD.CSV`, enters deep sleep. |
| `od-logger-lora32` | LilyGO T3 LoRa32 | LoRa + WiFi + 0.96" OLED + ADC + Web Master | LoRa32 telemetry node with onboard 0.96" SSD1306 display and Web Master streaming. |
| `i2c-scanner` | Natek-SD v2.0 | Standalone diagnostic | Scans I2C bus addresses (`0x3C`, `0x45`, `0x50`, `0x68`) and prints annotated device names to Serial. |

---

## 4. Operational Modes

The firmware operates in one of two fundamental architectures based on `#if defined(WEB_MASTER_MODE)`:

### Mode A: Web Master Mode (`WEB_MASTER_MODE`)
1. **Always Awake**: The ESP32 never enters deep sleep.
2. **IDLE State**:
   - Both `experimentActive` and `webMonitorActive` are `false`.
   - Sensor rails are powered off.
   - The device polls `GET /api/commands/pending` every **2000 ms**.
   - Signals entry into IDLE with a **3-pulse LED blink** (`ledSignalIdle()`).
3. **ACTIVE State (`experimentActive || webMonitorActive`)**:
   - Keeps sensor rails energized.
   - Samples sensors at the configured interval (`experimentIntervalMs` or `webMonitorIntervalMs`, default 2s or 5s).
   - If `experimentActive`, appends row to `/EXP_XX.CSV` on MicroSD in RFC 4180 format.
   - Updates SH1106 OLED live display.
   - Posts telemetry to `POST /api/telemetry/sensor`.
   - **Command Piggybacking**: Server responses to `POST /api/telemetry/sensor` carry any pending stop/control commands directly in the response body.
   - Transitions back to IDLE upon `stop_experiment` or `stop_monitor`.

### Mode B: Deep Sleep Cycling Mode (Field Production)
1. Wakes from deep sleep via DS3231 RTC interrupt or internal timer.
2. Samples OD sensor, RTC, and battery.
3. Appends reading directly to daily CSV `/YYYYMMDD.CSV` on MicroSD.
4. If WiFi available, connects and sends pending CSV rows to AquaControl.
5. Isolates GPIO pins via `gpio_hold_en()` and enters deep sleep for `moduleCfg.measure_time` minutes.
6. Signals deep sleep transition with a **2-pulse LED blink**.

---

## 5. Web Dashboard REST API Protocol (`AquaControl`)

Configured in [src/config.h](file:///c:/Users/Sam/Documents/Tesis/Codigos/od-logger/src/config.h):
- Host: `aquacontrol-two.vercel.app`
- Authentication Header: `X-Device-Key: <AQUACONTROL_DEVICE_KEY>`

### 1. Telemetry Upload (`POST /api/telemetry/sensor`)
Request JSON:
```json
{
  "datetime": "2026-09-12T05:14:00-05:00",
  "seconds_since_2000": 842512440,
  "water_temp_centi": 2500,
  "do_sat_deci_pct": 985,
  "do_milli_mg_l": 7850,
  "param3_centi": 120,
  "param4_centi": 0,
  "battery_mv": 3950,
  "rtc_temp_centi": 2450,
  "status": 0,
  "sent": true
}
```

Server Response with Piggybacked Command:
```json
{
  "success": true,
  "has_command": true,
  "pending_command": {
    "id": "2708b792-7f28-44dc-9d41-a1b7e408ecbb",
    "command_type": "stop",
    "payload": {
      "action": "stop_experiment",
      "experiment_id": "exp_1726117823901"
    }
  }
}
```

### 2. Idle Command Polling (`GET /api/commands/pending`)
Queried every 2000 ms during IDLE state. Returns the same payload structure as above.

### 3. Command Acknowledgment (`POST /api/commands/[id]/acknowledge`)
Request JSON:
```json
{
  "success": true,
  "sensor_state": {
    "monitor_active": false,
    "active_experiment": null
  }
}
```

### 4. Supported Control Actions
* `start_experiment`: Starts logging to dedicated SD CSV (`/EXP_XX.CSV`), activates continuous monitor at `interval_sec`.
* `stop_experiment`: Flushes and closes SD experiment file, turns off sensor power rail, enters IDLE state.
* `start_monitor` / `start`: Activates OLED & live telemetry stream without SD experiment file.
* `stop_monitor` / `stop`: Stops live telemetry stream, turns off sensor power rail, enters IDLE state.
* `set_sampling_rate`: Updates `experimentIntervalMs` / `webMonitorIntervalMs` (clamped between 1 and 60 seconds).
* `manual_sample`: Queues an immediate one-shot sample and telemetry upload.
* `sleep`: Puts device to sleep for `minutes` (or indefinite).

---

## 6. Coding Standards & Memory Rules (Ponytail Senior Dev Mode)

1. **Static Memory Control — NO Dynamic Heap Allocation**:
   - ArduinoJson 7 is installed in `lib/ArduinoJson`.
   - **Never** use unbounded dynamic `JsonDocument` in message handlers, as repeated `malloc`/`free` cycles fragment the ESP32 heap during 24/7 streaming.
   - Always use `StaticJsonDocument<N>` with explicit, verified capacities:
     - `StaticJsonDocument<1536> doc;` in `processTelemetryResponse()` (**mandatory**: do not reduce below 1536 or incoming server responses with UUIDs and metadata will fail with `NoMemory`).
     - `StaticJsonDocument<384> doc;` in `publishToAquaControl()`.
     - `StaticJsonDocument<384> pDoc;` in `onSensorCommandReceived()`.
     - `StaticJsonDocument<512> ackDoc;` in `acknowledgeSensorCommand()`.
     - `StaticJsonDocument<256> stateDoc;` for state JSON objects.
2. **Safe Logging Macros**:
   - Use `Log(...)`, `Logln(...)`, and `Logf(...)` from [src/log.h](file:///c:/Users/Sam/Documents/Tesis/Codigos/od-logger/src/log.h).
   - Never call `Serial.print()` directly in feature modules.
3. **SD Card FAT 8.3 Filenames**:
   - MicroSD driver uses `SdFat`. Always format filenames as `/XXXXXXXX.CSV` (8 chars max, uppercase, e.g. `/EXP_01.CSV`).
4. **Visual LED Signaling**:
   - `ledSignalIdle()`: 3 pulses (100 ms ON, 100 ms OFF) to indicate entry into IDLE.
   - Deep sleep transition: 2 pulses (100 ms ON, 100 ms OFF).

---

## 7. Build & Verification Commands

```powershell
# 1. Compile firmware for active web target
pio run -e od-logger-web

# 2. Flash to connected ESP32 and monitor serial output (pum)
pum -e od-logger-web

# 3. Compile and run protocol & experiment self-check test
g++ -std=c++17 test/test_experiment_flow.cpp -o test/test_experiment_flow.exe
./test/test_experiment_flow.exe
Remove-Item -Force test/test_experiment_flow.exe
```
