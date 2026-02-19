# Installation & Build Guide

This guide explains how to prepare, build, and flash the firmware for the
**LilyGO T-A7670SA Water Level Telemetry System**.
It also covers required libraries, partition configuration, and the complete
first-flash process.

---

# 1. Development Tools

You may use:

### Arduino IDE 2.x
Simple setup and easy library installation.

### PlatformIO
Advanced workflow, dependency management, CI/CD friendly.

---

# 2. Arduino IDE Setup

## 2.1 Install ESP32 Boards

1. Open Arduino IDE.
2. Go to: File → Preferences → Additional Boards Manager URLs
3. Add: https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json

4. Open Boards Manager and install **ESP32 by Espressif Systems**.

---

# 3. Required Libraries

Install via Library Manager:

- TinyGSM  
- PubSubClient  
- SdFat  
- ArduinoJson  
- Rtc by Makuna  
- ESP_SSLClient  
- ESP32 BLE Arduino  

---

# 4. ESP32 Partition Scheme (IMPORTANT)

Set:
Tools → Partition Scheme → **Huge APP (3MB No OTA / 1MB SPIFFS)**

# 5. Project Folder Structure
```
project/
│
├── n_level_gprs.ino
├── utilities.h
├── modem_mqtt.cpp/.h
├── power_sleep.cpp/.h
├── rtc_utils.cpp/.h
├── ble_manager.cpp/.h
├── eeprom_i2c.cpp/.h
├── error_sensor.cpp/.h
├── measurement_storage.cpp/.h

```


# 6. ThingsBoard / MQTT Configuration

Edit in:

src/modem_mqtt.cpp  
src/modem_mqtt.h

Set:
- APN
- MQTT Host
- Device Token

Telemetry topic:
v1/devices/me/telemetry

---

# 7. First Flash Procedure

1. Connect the LilyGO board.
2. Select **ESP32 Dev Module**.
3. Select correct COM port.
4. Ensure partition = Huge APP.
5. Ensure upload Speed = 921600 (or 115200). 
5. Upload firmware.

---

# 8. BLE Initial Configuration

Use nRF Connect or similar.

Device name:
**LILYGO_A7670_BLE**

Commands:
- T-1-10-2  
- D-2025-11-28-23-38-00  
