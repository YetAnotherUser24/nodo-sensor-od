#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

int main() {
  // 1. Basic milli-mg/L scaling tests
  float testValues[] = {0.0f, 0.001f, 8.456f, 14.205f, 20.000f, 32.760f};
  int16_t expected[] = {0, 1, 8456, 14205, 20000, 32760};

  for (size_t i = 0; i < sizeof(testValues) / sizeof(testValues[0]); ++i) {
    int16_t scaled = static_cast<int16_t>(std::roundf(testValues[i] * 1000.0f));
    assert(scaled == expected[i]);

    // Test reverse decoding
    float decoded = scaled / 1000.0f;
    assert(std::fabs(decoded - testValues[i]) < 0.0001f);

    // Test integer string formatting (as used in main.cpp & OLED)
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%d.%03d", scaled / 1000, std::abs(scaled % 1000));
    float parsed = std::strtof(buf, nullptr);
    assert(std::fabs(parsed - testValues[i]) < 0.001f);
  }

  // 2. Ensure int16_t capacity supports full typical DO range (0 - 30 mg/L -> 0 - 30000)
  assert(30000 <= INT16_MAX);

  std::printf("All OD milli-mg/L unit conversion checks passed successfully.\n");
  return 0;
}
