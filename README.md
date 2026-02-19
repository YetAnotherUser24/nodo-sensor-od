# LilyGO A7670SA – Water Level Telemetry System

Firmware for a **LilyGO T-A7670SA (ESP32 + SIM7670)** used as a low-power
**water level data logger** with:

- Water level sensor (multiple readings per measurement)
- External I2C EEPROM (compact ring buffer)
- SD card (human-readable CSV backup)
- RTC DS3231 (accurate timestamps)
- GPRS + MQTT (ThingsBoard compatible)
- BLE configuration channel
- Deep-sleep power management

The device can **keep logging offline** and later sync all pending data
to the cloud.

---

## Main features

- 📏 **Water level logging**
  - Multiple readings per measurement (noise reduction).
  - Basic sensor sanity checks and status bits.

- 💾 **Resilient local storage**
  - External EEPROM with header bitmap and per-record “sent” flag. 
  - Periodic flush of EEPROM pages to SD card as `YYYYMMDD_P.CSV`.
  - Conversion from CSV to JSON and sync to ThingsBoard via MQTT.

- 📡 **MQTT / ThingsBoard**
  - Publishes telemetry with **RTC-based timestamps** (epoch ms).
  - Sends water level array, battery, temperature, status bits and storage info (EEPROM free pages, SD free MB). 

- 📶 **Bluetooth Low Energy**
  - Single GATT characteristic for command/control.
  - Configuration of measurement/BLE/GPRS intervals via `T-<m>-<b>-<g>`.  
  - RTC time sync via `D-YYYY-MM-DD-HH-MM-SS`.
  - Live streaming of sensor data with `SEND`/`STOP`.

- ⏱️ **RTC + status flags**
  - Uses DS3231 for timekeeping and temperature.
  - Encodes sensor/SD/temperature issues into a 32-bit status field.

- 🔋 **Power management**
  - Battery voltage measurement using ADC and simple low-battery profile. 
  - Deep sleep between measurement / BLE / GPRS windows (handled by main sketch + `power_sleep`).

---

## Repository layout

- `src/n_level_gprs.ino`: main sketch (high-level state machine, setup/loop).
- `src/utilities.h`: common macros, board pin definitions, helpers.
- `src/modem_mqtt.*`: SIM7670 modem control and MQTT connection (AT, APN, TB tokens).
- `src/power_sleep.*`: power-saving logic (turning radios on/off, deep sleep).
- `src/rtc_utils.*`: DS3231 helpers (formatting timestamps, alarms).
- `src/ble_manager.*`: BLE service and command handler.   
- `src/eeprom_i2c.*`: low-level I2C EEPROM driver (byte/page/buffer ops, header reset).   
- `src/error_sensor.*`: error LED pattern + ISR for sensor events.   
- `src/measurement_storage.*`: measurement capture, EEPROM/SD storage, CSV/JSON conversion, telemetry sync.   

Docs:

- `docs/INSTALLATION.md`: how to build & flash (Arduino IDE / PlatformIO), required libraries.
- `docs/HARDWARE_SETUP.md`: wiring, SD/EEPROM, **resistor removal for UART debug**, first flash, partition scheme.
- `docs/ARCHITECTURE.md`: high-level design and module-level view.
- `docs/BLE_COMMANDS.md`: BLE protocol and examples.
- `docs/MQTT_RPC_GUIDE.md`: suggested RPC patterns for ThingsBoard.
- `docs/EEPROM_SD_FORMAT.md`: EEPROM page layout and SD CSV formats.

---

## Requirements

### Hardware

- LilyGO T-A7670SA (ESP32 + SIM7670 based board).
- SD card (formatted as FAT32, connected to `SD_MOSI/SD_MISO/SD_SCLK/SD_CS`).
- SIM card with data.
- Water level sensor (connected to dedicated serial + power pins as defined in `utilities.h` / main sketch).
- Li-ion or LiPo battery connected through board’s power input.

### Software

- **Arduino IDE** (2.x) or **PlatformIO**.
- Board support for **ESP32** (Espressif).
- Libraries (Arduino IDE → Library Manager):
  - `TinyGSM`
  - `PubSubClient`
  - `SdFat`
  - `ArduinoJson`
  - `Rtc by Makuna` (DS3231)
  - `ESP_SSLClient`
  - `ESP32 BLE Arduino`

See `docs/INSTALLATION.md` for a step-by-step guide.

---

## Quick start

1. Follow **hardware modifications** in `docs/HARDWARE_SETUP.md` if you need USB serial debug.
2. Install all required libraries from Arduino Library Manager.
3. In Arduino IDE:
   - Select "ESP32 dev module" as board.
   - Set **Partition Scheme** to **Huge APP (3MB)** (or equivalent).
   - Set Upload Speed to 921600 (or 115200). 
4. Configure:
   - APN / SIM credentials in `modem_mqtt.*`.
   - ThingsBoard token / broker URL.
5. Flash `src/n_level_gprs.ino`.
6. Open serial monitor and watch first boot logs.
7. Use BLE (see `docs/BLE_COMMANDS.md`) to:
   - Set intervals: `T-5-1-10`
   - Sync RTC: `D-2025-11-28-23-38-00`

After the first successful cycle, measurements will be stored in EEPROM, flushed to SD and eventually forwarded to ThingsBoard.

---

## Telemetry format (ThingsBoard style)

Typical telemetry JSON (via MQTT):

```json
{
  "ts": 1732837080000,
  "values": {
    "battery": 3840,
    "temperature": 2300,
    "status": 0,
    "waterLevelSensor": [123, 124, 121, 122, 123],
    "eepromFreePages": 1500,
    "sdFreeMB": 28700
  }
}
