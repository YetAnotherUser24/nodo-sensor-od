/**
 * @file oled_display.cpp
 * @brief 1.3" I2C SH1106 OLED visualization implementation for n-logger platform.
 */

#include "oled_display.h"
#include "board.h"
#include "log.h"
#include <Adafruit_GFX.h>
#include <Wire.h>

#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define OLED_I2C_ADDR_PRIMARY 0x3C
#define OLED_I2C_ADDR_SECONDARY 0x3D
#define OLED_PAGE_COUNT 8
#define OLED_BUF_SIZE (OLED_WIDTH * OLED_HEIGHT / 8) // 1024 bytes

/**
 * @brief Dedicated, rock-solid SH1106 1.3" 128x64 I2C driver subclassing Adafruit_GFX.
 */
class SH1106_Display : public Adafruit_GFX {
public:
  SH1106_Display() : Adafruit_GFX(OLED_WIDTH, OLED_HEIGHT), _i2cAddr(OLED_I2C_ADDR_PRIMARY) {
    memset(_buffer, 0, sizeof(_buffer));
  }

  bool begin(uint8_t i2cAddr = OLED_I2C_ADDR_PRIMARY) {
    _i2cAddr = i2cAddr;

    // Check if device acknowledges at this I2C address
    Wire.beginTransmission(_i2cAddr);
    if (Wire.endTransmission() != 0) {
      return false;
    }

    // Standard SH1106 1.3" initialization sequence with internal charge pump
    static const uint8_t init_cmds[] = {
        0xAE,         // 1. Display OFF
        0xD5, 0x80,   // 2. Set Display Clock Divide Ratio / Osc Freq (0x80)
        0xA8, 0x3F,   // 3. Set Multiplex Ratio (64 lines: 0x3F)
        0xD3, 0x00,   // 4. Set Display Offset (0x00)
        0x40,         // 5. Set Display Start Line (0)
        0xAD, 0x8B,   // 6. Set DC-DC Charge Pump Mode (0x8B = Internal DC-DC ON)
        0x8D, 0x14,   // 7. SSD1306 charge pump compat (0x14 = ON)
        0x33,         // 8. Set Charge Pump output voltage to 9.0V
        0xA1,         // 9. Set Segment Re-map (0xA1 = column 127 mapped to SEG0)
        0xC8,         // 10. Set COM Output Scan Direction (0xC8 = remapped)
        0xDA, 0x12,   // 11. Set COM Pins Hardware Config (0x12 = alternative)
        0x81, 0xCF,   // 12. Set Contrast Control (0xCF)
        0xD9, 0x22,   // 13. Set Pre-charge Period (0x22)
        0xDB, 0x40,   // 14. Set VCOM Deselect Level (0x40 = ~0.77*Vcc)
        0xA4,         // 15. Entire Display ON Resume (0xA4)
        0xA6,         // 16. Set Normal Display (0xA6 = non-inverted)
        0xAF          // 17. Display ON!
    };

    for (size_t i = 0; i < sizeof(init_cmds); i++) {
      sendCommand(init_cmds[i]);
    }

    clearDisplay();
    display();
    return true;
  }

  void drawPixel(int16_t x, int16_t y, uint16_t color) override {
    if (x < 0 || x >= OLED_WIDTH || y < 0 || y >= OLED_HEIGHT) {
      return;
    }

    uint16_t byteIdx = x + (y / 8) * OLED_WIDTH;
    uint8_t bitMask = 1 << (y & 7);

    if (color) {
      _buffer[byteIdx] |= bitMask;
    } else {
      _buffer[byteIdx] &= ~bitMask;
    }
  }

  void clearDisplay() {
    memset(_buffer, 0, sizeof(_buffer));
  }

  void display() {
    // Write 8 pages to SH1106 display RAM
    for (uint8_t page = 0; page < OLED_PAGE_COUNT; page++) {
      // 1. Set page address (0xB0 .. 0xB7)
      // 2. Set higher column address (0x10)
      // 3. Set lower column address (0x02 = 2-pixel offset for SH1106 132-column RAM)
      Wire.beginTransmission(_i2cAddr);
      Wire.write(0x00); // Command stream (Co=0, D/C#=0)
      Wire.write(0xB0 + page);
      Wire.write(0x10);
      Wire.write(0x02);
      Wire.endTransmission();

      // 4. Send 128 bytes of pixel data in 16-byte I2C chunks
      for (uint8_t col = 0; col < OLED_WIDTH; col += 16) {
        Wire.beginTransmission(_i2cAddr);
        Wire.write(0x40); // Data stream (Co=0, D/C#=1)
        Wire.write(&_buffer[page * OLED_WIDTH + col], 16);
        Wire.endTransmission();
      }
    }
  }

  void setContrast(uint8_t contrast) {
    sendCommand(0x81);
    sendCommand(contrast);
  }

  void sleep() {
    sendCommand(0xAE); // Display OFF
  }

  void sendCommand(uint8_t cmd) {
    Wire.beginTransmission(_i2cAddr);
    Wire.write(0x80); // Single command mode (Co=1, D/C#=0)
    Wire.write(cmd);
    Wire.endTransmission();
  }

private:
  uint8_t _i2cAddr;
  uint8_t _buffer[OLED_BUF_SIZE];
};

static SH1106_Display oledDisplay;
static bool oledInitialized = false;

extern bool bleDeviceConnected;

static bool oledProbeAttempted = false;

bool oledInit() {
  if (oledInitialized) {
    return true;
  }
  if (oledProbeAttempted) {
    return false;
  }
  oledProbeAttempted = true;

  // Ensure 3.3V I2C power rail is energized
  setPinActive(I2C_EN_PIN, I2C_EN_ACTIVE_HIGH);
  delay(50); // Allow 3.3V power rail and display controller internal POR to stabilize

  // Try primary address 0x3C first, then 0x3D fallback
  if (oledDisplay.begin(OLED_I2C_ADDR_PRIMARY)) {
    oledInitialized = true;
  } else if (oledDisplay.begin(OLED_I2C_ADDR_SECONDARY)) {
    oledInitialized = true;
  } else {
    oledInitialized = false;
#ifdef DEBUG
    Logln("[OLED] SH1106 display not detected on I2C bus (checked 0x3C, 0x3D).");
#endif
    return false;
  }

  oledDisplay.clearDisplay();
  oledDisplay.setTextColor(1);
  oledDisplay.setTextSize(1);
  oledDisplay.setCursor(0, 0);
  oledDisplay.display();

#ifdef DEBUG
  Logln("[OLED] 1.3\" SH1106 OLED initialized successfully.");
#endif

  return true;
}

bool isOledAvailable() {
  return oledInitialized;
}

void oledShowMonitorStart(uint32_t intervalSec) {
  if (!oledInitialized && !oledInit()) {
    return;
  }

  // Reinforce I2C power rail
  setPinActive(I2C_EN_PIN, I2C_EN_ACTIVE_HIGH);

  oledDisplay.clearDisplay();

  // Top banner (Inverted box)
  oledDisplay.fillRect(0, 0, OLED_WIDTH, 12, 1);
  oledDisplay.setTextColor(0, 1);
  oledDisplay.setTextSize(1);
  oledDisplay.setCursor(4, 2);
  oledDisplay.print("MONITORING ACTIVE");

  oledDisplay.setTextColor(1);
  oledDisplay.setCursor(0, 18);
  oledDisplay.printf("Sensor: %s", SENSOR_NAME);
  oledDisplay.setCursor(0, 30);
  oledDisplay.printf("Interval: %u s", (unsigned int)intervalSec);
  oledDisplay.setCursor(0, 44);
  oledDisplay.print("Sampling started...");
  oledDisplay.drawFastHLine(0, 58, OLED_WIDTH, 1);

  oledDisplay.display();
}

void oledUpdateMonitor(const TelemetryPayload& tp, const RtcDateTime& dt, uint32_t intervalSec) {
  if (!oledInitialized && !oledInit()) {
    return;
  }

  // Reinforce I2C power rail
  setPinActive(I2C_EN_PIN, I2C_EN_ACTIVE_HIGH);

  oledDisplay.clearDisplay();

  // Top header bar (Inverted for visual clarity)
  oledDisplay.fillRect(0, 0, OLED_WIDTH, 11, 1);
  oledDisplay.setTextColor(0, 1);
  oledDisplay.setTextSize(1);
  oledDisplay.print("MONITOR: OD DIGISENS");

  // Measurement readings
  oledDisplay.setTextColor(1);

  // Row 1 (y = 13): Time, BLE status & Interval
  oledDisplay.setCursor(0, 13);
  oledDisplay.printf("%02u:%02u:%02u %s Int:%us", (unsigned int)dt.Hour(), (unsigned int)dt.Minute(),
                    (unsigned int)dt.Second(), bleDeviceConnected ? "[BLE]" : "[ADV]",
                    (unsigned int)intervalSec);

  float doValue = tp.sensorReadings[1] / 1000.0f;
  float satValue = tp.sensorReadings[2] / 10.0f;
  float tempValue = tp.sensorReadings[0] / 100.0f;
  float p3Value = tp.sensorReadings[3] / 100.0f;

  oledDisplay.setCursor(0, 23);
  oledDisplay.printf("DO:   %.3f mg/L", doValue);

  oledDisplay.setCursor(0, 33);
  oledDisplay.printf("Sat:  %.1f %%Sat", satValue);

  oledDisplay.setCursor(0, 43);
  oledDisplay.printf("Temp: %.2f C", tempValue);

  oledDisplay.setCursor(0, 53);
#if defined(SENSOR_BATTERY_ADC) || defined(SENSOR_INA226)
  oledDisplay.printf("Bat:%.2fV P3:%.1f", tp.battery / 1000.0f, p3Value);
#else
  oledDisplay.printf("Status: 0x%08X", (unsigned int)tp.status);
#endif

  oledDisplay.display();
}

void oledShowMonitorStop() {
  if (!oledInitialized) {
    return;
  }

  oledDisplay.clearDisplay();

  oledDisplay.fillRect(0, 0, OLED_WIDTH, 12, 1);
  oledDisplay.setTextColor(0, 1);
  oledDisplay.setTextSize(1);
  oledDisplay.setCursor(4, 2);
  oledDisplay.print("MONITORING STOPPED");

  oledDisplay.setTextColor(1);
  oledDisplay.setCursor(0, 24);
  oledDisplay.print("Continuous monitoring");
  oledDisplay.setCursor(0, 36);
  oledDisplay.print("disabled (STOP).");
  oledDisplay.setCursor(0, 50);
  oledDisplay.print("Entering standby/sleep.");

  oledDisplay.display();
  delay(1000);
  oledPowerOff();
}

void oledPowerOff() {
  if (!oledInitialized) {
    return;
  }

  oledDisplay.sleep();
  oledInitialized = false;
  setPinInactive(I2C_EN_PIN, I2C_EN_ACTIVE_HIGH);
}
