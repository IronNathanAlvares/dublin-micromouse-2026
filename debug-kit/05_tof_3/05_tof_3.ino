// =====================================================================
// STEP 5 - Three ToF distance sensors (LEFT + FRONT + RIGHT)
// =====================================================================
// WIRING
//   All ToF sensors share power and I2C:
//     VIN -> ESP32 3V3        GND -> ESP32 GND
//     SDA -> ESP32 GPIO6      SCL -> ESP32 GPIO7
//   Each sensor gets its OWN XSHUT wire (this is how they're told apart):
//     LEFT  sensor XSHUT -> GPIO18   (already wired in step 3)
//     FRONT sensor XSHUT -> GPIO19   (already wired in step 4)
//     RIGHT sensor XSHUT -> GPIO20   (new)
//   The IMU from step 2 can stay connected - it doesn't clash.
//
// WHAT YOU'LL SEE
//   Serial Monitor when it works:
//       STEP 5: three ToF sensors
//         ToF L  XSHUT=GPIO18  addr=0x30  PASS
//         ToF F  XSHUT=GPIO19  addr=0x31  PASS
//         ToF R  XSHUT=GPIO20  addr=0x29  PASS
//       3/3 sensors OK
//       ------------
//       L:142 mm  F:310 mm  R:98 mm
//       L:---     F:305 mm  R:101 mm     <- "---" = nothing in range (normal)
//   Wave a hand in front of each sensor in turn: only THAT one's number
//   should change. If the wrong one changes, its XSHUT wires are swapped.
//   If one fails, it shows FAIL at startup (with what to check), and
//   L:FAIL / F:FAIL / R:FAIL on every line after.
//
// LIBRARY: Library Manager -> "VL53L0X" by Pololu (NOT Adafruit_VL53L0X)
// =====================================================================

#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>

constexpr uint8_t PIN_SDA = 6;
constexpr uint8_t PIN_SCL = 7;

// At or past this = "no wall in range" (the sensor reports ~8190).
constexpr uint16_t MAX_VALID_MM = 2000;

struct Tof {
  const char* name;
  uint8_t     xshut;   // GPIO wired to this sensor's XSHUT pin
  uint8_t     addr;    // I2C address it gets moved to
  VL53L0X     sensor;
  bool        ok;
};

Tof tofs[] = {
  {"L", 18, 0x30, VL53L0X(), false},
  {"F", 19, 0x31, VL53L0X(), false},
  {"R", 20, 0x29, VL53L0X(), false},  // last one up keeps the default address
};
constexpr uint8_t NUM_TOF = sizeof(tofs) / sizeof(tofs[0]);

// Every VL53L0X wakes up at 0x29, so they're woken one at a time and each
// moves to its own address before the next one wakes.
bool startTof(Tof& t) {
  digitalWrite(t.xshut, HIGH);
  delay(10);
  t.sensor.setTimeout(100);
  if (!t.sensor.init()) {
    // Back to sleep, or it would clash with the next sensor at 0x29.
    digitalWrite(t.xshut, LOW);
    return false;
  }
  if (t.addr != 0x29) t.sensor.setAddress(t.addr);
  t.sensor.startContinuous();
  return true;
}

void setup() {
  Serial.begin(115200);
  const uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 2000) {}
  Serial.println("\nSTEP 5: three ToF sensors");

  for (auto& t : tofs) {  // every sensor asleep first
    pinMode(t.xshut, OUTPUT);
    digitalWrite(t.xshut, LOW);
  }
  delay(10);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);

  uint8_t passed = 0;
  for (auto& t : tofs) {
    t.ok = startTof(t);
    Serial.printf("  ToF %s  XSHUT=GPIO%u  addr=0x%02X  %s\n",
                  t.name, t.xshut, t.addr, t.ok ? "PASS" : "FAIL");
    if (t.ok) passed++;
    else Serial.printf("    -> check VIN->3V3, GND, SDA/SCL, and XSHUT->GPIO%u\n", t.xshut);
  }
  Serial.printf("%u/%u sensors OK\n------------\n", passed, NUM_TOF);
}

void loop() {
  for (auto& t : tofs) {
    Serial.printf("%s:", t.name);
    if (!t.ok) { Serial.print("FAIL    "); continue; }
    const uint16_t mm = t.sensor.readRangeContinuousMillimeters();
    if (t.sensor.timeoutOccurred() || mm >= MAX_VALID_MM) Serial.print("---     ");
    else Serial.printf("%-4umm  ", mm);
  }
  Serial.println();
  delay(100);  // ~10 Hz
}
