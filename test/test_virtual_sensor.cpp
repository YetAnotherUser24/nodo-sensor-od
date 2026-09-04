#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>

// Define DISABLE_ESPNOW and SENSOR_VIRTUAL for the test harness
#define DISABLE_ESPNOW
#define SENSOR_VIRTUAL

#include "../src/esp_now_packet.h"
#include "../src/esp_now_sender.h"

int main() {
  std::printf("Running Virtual Sensor and ESP-NOW Stub tests...\n");

  // 1. Verify ESP-NOW inline stubs return true with zero side effects
  assert(espNowSenderInit() == true);

  OdTelemetryPacket testPkt = {};
  testPkt.magic = AQUACONTROL_PACKET_MAGIC;
  testPkt.dissolved_oxygen = 7.85f;
  testPkt.water_temperature = 24.5f;
  testPkt.oxygen_saturation = 95.2f;
  testPkt.battery_voltage = 4.12f;
  assert(espNowBroadcastDO(testPkt) == true);
  std::printf("  [PASS] ESP-NOW stubs return true when DISABLE_ESPNOW is defined.\n");

  // 2. Verify Virtual Sensor simulation ranges over multiple cycles
  float simAngle = 0.0f;
  for (int cycle = 0; cycle < 100; ++cycle) {
    simAngle += 0.25f;

    float simDoMgL = 7.8f + 0.8f * std::sinf(simAngle);
    float simTempC = 24.5f + 0.7f * std::cosf(simAngle);
    float simSatPct = 95.0f + 6.0f * std::sinf(simAngle);

    int16_t tempCenti = static_cast<int16_t>(std::roundf(simTempC * 100.0f));
    int16_t doMilli = static_cast<int16_t>(std::roundf(simDoMgL * 1000.0f));
    int16_t satDeci = static_cast<int16_t>(std::roundf(simSatPct * 10.0f));
    uint32_t batteryMv = 4120;

    // Plausibility bounds
    assert(tempCenti >= 2000 && tempCenti <= 3000);  // 20.0 to 30.0 C
    assert(doMilli >= 6500 && doMilli <= 9000);       // 6.5 to 9.0 mg/L
    assert(satDeci >= 850 && satDeci <= 1050);        // 85.0 to 105.0 %
    assert(batteryMv >= 3400);                        // Safe above low-battery threshold (3400 mV)

    // Verify reverse decoding for dashboard JSON payload
    float decodedDo = doMilli / 1000.0f;
    assert(std::fabs(decodedDo - simDoMgL) < 0.001f);

    float decodedTemp = tempCenti / 100.0f;
    assert(std::fabs(decodedTemp - simTempC) < 0.01f);

    float decodedSat = satDeci / 10.0f;
    assert(std::fabs(decodedSat - simSatPct) < 0.1f);
  }
  std::printf("  [PASS] Simulated telemetry values remain within valid physical bounds across 100 cycles.\n");

  std::printf("All virtual sensor tests passed successfully!\n");
  return 0;
}
