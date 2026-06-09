# Implementation Plan

[Overview]
Refactor the single-purpose "n-level" ultrasonic water level datalogger into a modular "n-logger" multi-sensor platform with compile-time togglable features (SD, LTE Modem) and PlatformIO build environments for different sensor configurations.

The current codebase is a monolithic Arduino sketch (~657 lines in `n_level_gprs.ino`) tightly coupled to a MaxBotix ultrasonic sensor via UART. Supporting modules (`measurement_storage.cpp`, `ble_manager.cpp`, `modem_mqtt.cpp`, etc.) contain duplicated `ModuleConfig` struct definitions and fragile chains of `extern` declarations referencing globals defined in the .ino file. The platformio.ini defines only a single `esp32dev` environment with feature flags (#define SD_MODULE, BT_MODULE, GPRS_MODULE) hardcoded in the .ino rather than passed as build flags.

This refactoring will:
- Rename the project identity from "n-level" to "n-logger" (configurable per build)
- Extract sensor-specific logic into a compile-time-switched sensor abstraction layer using `#ifdef` blocks
- Move feature toggles (SD_MODULE, GPRS_MODULE) from the .ino into platformio.ini build_flags
- Convert the monolithic .ino into a lightweight main.cpp that composes modular components
- Consolidate shared types (ModuleConfig, TelemetryPayload, status bits, EEPROM layout) into a single header to eliminate duplication and fragile extern chains
- Define multiple PlatformIO build environments for different sensor configurations
- Keep BLE and EEPROM modules always included (non-toggleable)

[Types]
Introduce a unified type system in a single shared header to eliminate duplication across modules.

New file: `src/n_logger_config.h` — Central type and constant definitions, replacing duplicated structs and defines scattered across 4+ files.

Contents:
```cpp
// Device identity (overridable per build via -DN_LOGGER_DEVICE_NAME=...)
#ifndef N_LOGGER_DEVICE_NAME
#define N_LOGGER_DEVICE_NAME "n-logger"
#endif
#ifndef CODE_VERSION
#define CODE_VERSION 0x02
#endif

// Timestamp epoch base
#ifndef UNIX_OFFSET_2000
#define UNIX_OFFSET_2000 946684800UL
#endif

// EEPROM geometry (always included)
#ifndef EEPROM_ADDRESS
#define EEPROM_ADDRESS 0x50
#endif
#ifndef EEPROM_PAGE_SIZE
#define EEPROM_PAGE_SIZE 32
#endif
#ifndef EEPROM_HEADER_SIZE
#define EEPROM_HEADER_SIZE 8
#endif
#ifndef EEPROM_MAX_PAGES
#define EEPROM_MAX_PAGES 2048
#endif
#ifndef EEPROM_FLUSH_AFTER
#define EEPROM_FLUSH_AFTER 288
#endif
#ifndef EEPROM_SENT_FLAG_OFFSET
#define EEPROM_SENT_FLAG_OFFSET (EEPROM_PAGE_SIZE - 1)
#endif
#define FLAG_NOT_SENT 0x00
#define FLAG_SENT     0x01

// Runtime module scheduling configuration
struct ModuleConfig {
  int measure_time;
  int bluetooth_time;
  int GPRS_time;
};

// Telemetry payload structure
struct TelemetryPayload {
  int16_t  freeStorage;
  int16_t  temperature;
  int16_t  humidity;
  int16_t  waterLevelSensor[NUM_READINGS];  // Will be renamed to sensorReadings
  uint32_t battery;
  uint32_t status;
};

// Status bit definitions
#define STATUS_INTERNET_BIT           0
#define STATUS_FULL_MEMORY_BIT        1
#define STATUS_CORRUPTED_MEMORY_BIT   2
#define STATUS_CODE_ERROR_BIT         3
#define STATUS_WATER_LEVEL_SENSOR_BIT 16
#define STATUS_SD_CARD_BIT            17
#define STATUS_TEMPERATURE_BIT        18
#define STATUS_GRPS_BIT               19

// Transmission recovery
#define MAX_FAILED_TRANSMISSIONS 3
```

Modify `config_template.h` → `config_template.h` additions:
- Add `SENSOR_TYPE` define (0=none, 1=MaxBotix UART, 2=I2C Temperature, 3=Radar, etc.)
- Add sensor-specific pin/parameter defines gated by `#if SENSOR_TYPE == N`
- Add `N_LOGGER_DEVICE_NAME` default (overridable per build)

Modify `TelemetryPayload`:
- Rename `waterLevelSensor[NUM_READINGS]` to `sensorReadings[MAX_SENSOR_READINGS]` to be sensor-agnostic
- Add `#ifndef MAX_SENSOR_READINGS` / `#define MAX_SENSOR_READINGS 5` (default 5, overridable per sensor type)

[Files]
Create new files, modify existing files, and restructure the project for modularity.

**New files:**
- `src/n_logger_config.h` — Central type definitions, status bits, EEPROM constants, ModuleConfig, TelemetryPayload (as detailed in [Types])
- `src/sensor_engine.h` — Sensor abstraction layer header: declares `sensorInit()`, `sensorRead()`, `sensorGetType()`, `sensorWarmupMs()`
- `src/sensor_engine.cpp` — Sensor implementation using `#if SENSOR_TYPE == ...` blocks. Contains:
  - `#if SENSOR_TYPE == 1` (MaxBotix UART): existing `WaterLevelRead()`, UART init on pins SENSOR_RX_PIN/SENSOR_TX_PIN
  - `#if SENSOR_TYPE == 2` (I2C Temperature): placeholder for I2C temp sensor (e.g., SHT30/BME280)
  - `#if SENSOR_TYPE == 3` (Radar): placeholder for radar distance sensor
  - `#if SENSOR_TYPE == 0` (None): stub that returns 0 / no-op
  - A `sensorFillReadings(TelemetryPayload& tp)` function that populates `tp.sensorReadings[]` with `NUM_READINGS` samples
- `src/main.cpp` — Replaces `n_level_gprs.ino`. Lightweight entry point that:
  - Includes `n_logger_config.h` and all module headers
  - Defines all remaining globals (device_ID, eepromWritePage, bootCount, state flags, BLE objects, modem objects, SD objects, RTC objects, etc.)
  - Contains `setup()` and `loop()` with same logic flow but using modular calls
  - References `sensor_engine.h` instead of directly calling `WaterLevelRead()`

**Modified files:**
- `src/measurement_storage.cpp`:
  - Remove duplicated `ModuleConfig`, `TelemetryPayload` structs (now in n_logger_config.h)
  - Remove all `#ifndef` fallback defines for EEPROM_*, FLAG_*, STATUS_*, SD_* constants (now in n_logger_config.h)
  - Replace 20-line `extern` chain with `#include "n_logger_config.h"`
  - Replace direct `WaterLevelRead()` and `measuredVbat` references with `sensorFillReadings(telemetryPayload)` and generic sensor API calls
  - Update EEPROM page packing to use `sensorReadings` instead of `waterLevelSensor`
- `src/measurement_storage.h`:
  - Replace `NUM_READINGS` references with `MAX_SENSOR_READINGS`
  - Update docstrings from "water level" to "sensor"
- `src/modem_mqtt.cpp`:
  - Remove duplicated `ModuleConfig` struct
  - Remove local `#define` for pins/LED (now in n_logger_config.h)
  - Change serial banner from "N-LEVEL" to `N_LOGGER_DEVICE_NAME`
  - Guard entire modem-related code with `#ifdef GPRS_MODULE`
  - Update MQTT client ID from "lilygoClient" to configurable via `MQTT_CLIENT_ID`
- `src/modem_mqtt.h`:
  - Add `#ifdef GPRS_MODULE` guards around all function declarations (stub when disabled)
- `src/ble_manager.cpp`:
  - Remove duplicated `ModuleConfig` struct
  - Remove local `#define` for pins (now in n_logger_config.h)
  - Change BLE device name from "LILYGO_A7670_BLE" to configurable via build flag
- `src/ble_manager.h`:
  - No structural changes needed (BLE always included)
- `src/power_sleep.cpp`:
  - Guard SD bus suspend/resume with `#ifdef SD_MODULE`
  - Guard modem sleep with `#ifdef GPRS_MODULE`
- `src/power_sleep.h`:
  - No structural changes needed
- `src/eeprom_i2c.h` / `src/eeprom_i2c.cpp`:
  - No structural changes (always included)
- `src/error_sensor.h` / `src/error_sensor.cpp`:
  - No structural changes (always included)
- `src/rtc_utils.h` / `src/rtc_utils.cpp`:
  - No structural changes (always included)
- `src/utilities.h`:
  - No structural changes (LilyGO board definitions)
- `src/config_template.h`:
  - Add `SENSOR_TYPE` define (default 1 for MaxBotix)
  - Add sensor-specific sections gated by `#if SENSOR_TYPE`
  - Add `#ifndef N_LOGGER_DEVICE_NAME` / `#define N_LOGGER_DEVICE_NAME "n-logger"`
  - Add `#ifndef MQTT_CLIENT_ID` / `#define MQTT_CLIENT_ID "nLoggerClient"`
  - Add `MAX_SENSOR_READINGS` (default 5)

**Deleted files:**
- `src/n_level_gprs.ino` — Replaced by `src/main.cpp`

**PlatformIO configuration (`platformio.ini`):**
Rewrite to have multiple build environments with differentiated build_flags:

```ini
[common]
platform = espressif32
framework = arduino
monitor_speed = 115200
upload_speed = 921600
board_build.partitions = huge_app.csv
board_build.flash_mode = dio
lib_extra_dirs = lib
build_flags_common =
    -DBOARD_BAT_ADC_PIN=35
    -DCORE_DEBUG_LEVEL=0
    -DN_LOGGER_DEVICE_NAME=\"n-logger\"
    -DMQTT_CLIENT_ID=\"nLoggerClient\"

[env:n-logger-maxbotix-sd-ble-gprs]
board = esp32dev
build_flags =
    ${common.build_flags_common}
    -DLILYGO_T_A7670
    -DDEBUG
    -DMODEM_BAUDRATE=115200
    -DTINY_GSM_MODEM_SIM7600
    -DTINY_GSM_RX_BUFFER=1024
    -DSENSOR_TYPE=1
    -DSD_MODULE
    -DGPRS_MODULE

[env:n-logger-maxbotix-sd-ble]
board = esp32dev
build_flags =
    ${common.build_flags_common}
    -DLILYGO_T_A7670
    -DDEBUG
    -DSENSOR_TYPE=1
    -DSD_MODULE

[env:n-logger-maxbotix-ble-gprs]
board = esp32dev
build_flags =
    ${common.build_flags_common}
    -DLILYGO_T_A7670
    -DDEBUG
    -DMODEM_BAUDRATE=115200
    -DTINY_GSM_MODEM_SIM7600
    -DTINY_GSM_RX_BUFFER=1024
    -DSENSOR_TYPE=1
    -DGPRS_MODULE

[env:n-logger-i2ctemp-sd-ble-gprs]
board = esp32dev
build_flags =
    ${common.build_flags_common}
    -DLILYGO_T_A7670
    -DDEBUG
    -DMODEM_BAUDRATE=115200
    -DTINY_GSM_MODEM_SIM7600
    -DTINY_GSM_RX_BUFFER=1024
    -DSENSOR_TYPE=2
    -DSD_MODULE
    -DGPRS_MODULE

[env:n-logger-radar-sd-ble-gprs]
board = esp32dev
build_flags =
    ${common.build_flags_common}
    -DLILYGO_T_A7670
    -DDEBUG
    -DMODEM_BAUDRATE=115200
    -DTINY_GSM_MODEM_SIM7600
    -DTINY_GSM_RX_BUFFER=1024
    -DSENSOR_TYPE=3
    -DSD_MODULE
    -DGPRS_MODULE
```

[Functions]
Modify existing functions and introduce new sensor abstraction functions.

**New functions:**
- `void sensorInit()` in `sensor_engine.cpp` — Initializes the sensor hardware based on `SENSOR_TYPE`. For type 1 (MaxBotix): configures UART pins as inputs, calls `SensorSerial.begin()`. For type 2 (I2C temp): initializes I2C sensor. For type 0: no-op.
- `int16_t sensorReadSingle()` in `sensor_engine.cpp` — Reads one sample from the active sensor. For type 1: calls the existing `WaterLevelRead()` logic. For other types: TBD stubs returning 0.
- `void sensorFillReadings(TelemetryPayload& tp)` in `sensor_engine.cpp` — Collects `NUM_READINGS` samples into `tp.sensorReadings[]` with timeout and warmup delay. Extracts the sampling loop currently inside `takeMeasurement()`.
- `int sensorGetType()` in `sensor_engine.cpp` — Returns `SENSOR_TYPE` compile-time constant.
- `uint16_t sensorWarmupMs()` in `sensor_engine.cpp` — Returns sensor-specific warmup time in ms (MaxBotix: 175ms, others: TBD).
- `void sensorPowerOn()` in `sensor_engine.cpp` — Powers on sensor via SENSOR_PWR_PIN.
- `void sensorPowerOff()` in `sensor_engine.cpp` — Powers off sensor.
- `void sensorSerialEnd()` in `sensor_engine.cpp` — Ends sensor serial communication.

**Modified functions:**
- `takeMeasurement()` in `measurement_storage.cpp`:
  - Replace inline sensor sampling loop with call to `sensorFillReadings(telemetryPayload)`
  - Replace `waterLevelSensor` field accesses with `sensorReadings`
  - Replace direct `SensorSerial.begin()`, `digitalWrite(18, HIGH/LOW)`, `SensorSerial.end()` with `sensorInit()`, `sensorPowerOn()`, `sensorPowerOff()`, `sensorSerialEnd()`
  - Update debug serial output to use sensor-agnostic naming
  - Update EEPROM page layout comments to reflect generic sensor readings
- `setup()` in `main.cpp`:
  - Replace "WELCOME TO N-LEVEL" with `N_LOGGER_DEVICE_NAME` banner
  - Replace inline `#define SD_MODULE true` / `#define GPRS_MODULE true` with `#ifdef` checks on build flags
  - Guard SD init with `#ifdef SD_MODULE`
  - Guard GPRS/modem init with `#ifdef GPRS_MODULE`
  - Update serial output to show sensor type name based on `SENSOR_TYPE`
- `loop()` in `main.cpp`:
  - Guard `handleBleLedPattern()` with `#ifdef BT_MODULE` check (BLE always included, but LED pattern may reference SD/GPRS status bits)
- `sendSDPendingToThingsboard()` in `measurement_storage.cpp`:
  - Wrap entire function body with `#ifdef SD_MODULE` / `#ifdef GPRS_MODULE` guards
- `sendEEPROMToThingsboard()` in `measurement_storage.cpp`:
  - Replace `waterLevelSensor` JSON key with `sensorReadings`
  - Guard MQTT publish with `#ifdef GPRS_MODULE`
- `csvLineToJson()` in `measurement_storage.cpp`:
  - Replace `waterLevelSensor` JSON key with `sensorReadings`
- `dumpEEPROM()` in `measurement_storage.cpp`:
  - Guard SD init and file operations with `#ifdef SD_MODULE`
- `writeEEPROMLine()` in `measurement_storage.cpp`:
  - Guard SD file operations with `#ifdef SD_MODULE`
- `getSDFreeMB()` in `measurement_storage.cpp`:
  - Guard entire function body with `#ifdef SD_MODULE`, return 0 when disabled
- `initA7670()` in `modem_mqtt.cpp`:
  - Wrap entire function with `#ifdef GPRS_MODULE`
- `sleepA7670()` in `modem_mqtt.cpp`:
  - Wrap entire function with `#ifdef GPRS_MODULE`
- `connectMQTT()` in `modem_mqtt.cpp`:
  - Replace hardcoded client ID "lilygoClient" with `MQTT_CLIENT_ID`
  - Wrap with `#ifdef GPRS_MODULE`
- `processQueuedRpcCommands()` in `modem_mqtt.cpp`:
  - Wrap with `#ifdef GPRS_MODULE`
- `publishJsonLineToTB()` in `modem_mqtt.cpp`:
  - Wrap with `#ifdef GPRS_MODULE`
- `onMqttMessage()` in `modem_mqtt.cpp`:
  - Wrap with `#ifdef GPRS_MODULE`
- `prepareForDeepSleep()` in `power_sleep.cpp`:
  - Guard SD suspend with `#ifdef SD_MODULE`
  - Guard modem sleep call with `#ifdef GPRS_MODULE`
- `sdBusSuspend()` / `sdBusResume()` in `power_sleep.cpp`:
  - Add `#ifdef SD_MODULE` guards to function bodies
- `boardRadiosOff()` in `power_sleep.cpp`:
  - Guard WiFi/BT turn-off with `#ifdef GPRS_MODULE` (radios needed for modem)
- `startBLE()` in `ble_manager.cpp`:
  - Change device name from `BLE_DEVICE_NAME` to configurable `N_LOGGER_BLE_NAME`
- `WaterLevelRead()` in `n_level_gprs.ino`:
  - Move into `sensor_engine.cpp` under `#if SENSOR_TYPE == 1` block
  - Rename to `sensorReadMaxbotix()` (internal to sensor_engine.cpp)

**Removed functions:**
- `extractJSON()` (line 580-583 in n_level_gprs.ino) — Unused anywhere in the codebase, safe to remove
- `changeBit()` (line 595-605 in n_level_gprs.ino) — Move to `utilities.h` as inline utility function (used by multiple modules)

[Classes]
No new classes (following the `#ifdef` approach per user preference). The sensor abstraction uses compile-time polymorphism via `#if SENSOR_TYPE` blocks rather than virtual class hierarchies.

**Modified classes:**
- `MyCallbacks` in `ble_manager.cpp` — No API changes, but references to `WaterLevelRead()` removed in favor of generic sensor streaming
- `ServerConnectionCallbacks` in `ble_manager.cpp` — No changes needed

**Removed classes:**
None.

[Dependencies]
No new library dependencies. Existing libraries remain:
- TinyGsmClient (local lib/)
- PubSubClient (local lib/)
- SdFat (local lib/) — now conditionally compiled via `#ifdef SD_MODULE`
- Rtc_by_Makuna (local lib/)
- ArduinoJson (local lib/)
- ESP_SSLClient (local lib/)
- TinyGPSPlus (local lib/)

The PlatformIO `lib_deps` section can remain commented out since all libraries are local. The `TinyGSM` dependency is now conditionally included only when `GPRS_MODULE` is defined — achieved via `#ifdef` guards in source files rather than modifying the build system's library linking.

[Testing]
Testing approach for the refactoring:

1. **Compile verification**: Build each PlatformIO environment (`n-logger-maxbotix-sd-ble-gprs`, `n-logger-maxbotix-sd-ble`, `n-logger-maxbotix-ble-gprs`, `n-logger-i2ctemp-sd-ble-gprs`, `n-logger-radar-sd-ble-gprs`) and verify zero compilation errors
2. **Feature toggle verification**: Confirm that:
   - Building without `-DSD_MODULE` produces a binary with no SdFat references (smaller flash)
   - Building without `-DGPRS_MODULE` produces a binary with no TinyGSM/modem references (smaller flash)
   - EEPROM and BLE code always present regardless of flags
3. **Functional equivalence**: For the `n-logger-maxbotix-sd-ble-gprs` environment (full feature equivalent of current single build), verify:
   - Same EEPROM page format (32 bytes, same layout)
   - Same CSV format
   - Same MQTT JSON payload structure (with `sensorReadings` key replacing `waterLevelSensor`)
   - Same BLE command protocol (T-x-y-z, D-YYYY-MM-DD-HH-MM-SS, SEND, STOP)
   - Same RPC command handling
4. **Documentation update**: Update `docs/ARCHITECTURE.md` to reflect new module structure and sensor abstraction

Test files location: `test/README` already exists; manual build verification is sufficient since this is an embedded Arduino project without a unit test framework.

[Implementation Order]
Implement changes in dependency order to minimize conflicts and ensure each step produces a compilable intermediate state.

1. **Create `src/n_logger_config.h`** — Central type definitions replacing all duplicated structs and constants
2. **Update `src/config_template.h`** — Add sensor type, device name, and MQTT client ID defines
3. **Create `src/sensor_engine.h` and `src/sensor_engine.cpp`** — Sensor abstraction layer with MaxBotix implementation (type 1) and stubs for types 0/2/3
4. **Update `src/measurement_storage.h`** — Replace NUM_READINGS with MAX_SENSOR_READINGS, update docstrings
5. **Update `src/measurement_storage.cpp`** — Remove duplicates, use n_logger_config.h, use sensor API, add SD_MODULE/GPRS_MODULE guards, rename waterLevelSensor to sensorReadings
6. **Update `src/modem_mqtt.h`** — Add GPRS_MODULE guards around declarations
7. **Update `src/modem_mqtt.cpp`** — Remove duplicates, add GPRS_MODULE guards, use n_logger_config.h, update MQTT client ID
8. **Update `src/ble_manager.cpp`** — Remove duplicated struct, use n_logger_config.h, update BLE device name
9. **Update `src/power_sleep.cpp`** — Add SD_MODULE and GPRS_MODULE guards
10. **Create `src/main.cpp`** — New entry point replacing n_level_gprs.ino, composing all modules with feature guards
11. **Rewrite `platformio.ini`** — Multiple build environments with differentiated build flags
12. **Delete `src/n_level_gprs.ino`** — Replaced by main.cpp
13. **Update `docs/ARCHITECTURE.md`** — Reflect new module structure and naming