// =====================================================================
// STEP 3 - One ToF distance sensor (LEFT)
// =====================================================================
// WIRING
//   All ToF sensors share power and I2C:
//     VIN -> ESP32 3V3        GND -> ESP32 GND
//     SDA -> ESP32 GPIO6      SCL -> ESP32 GPIO7
//   Each sensor gets its OWN XSHUT wire (this is how they're told apart):
//     LEFT  sensor XSHUT -> GPIO18
//   Only connect the sensors listed here. An extra sensor with its XSHUT
//   unconnected wakes up at 0x29 on its own and breaks the others.
//   The IMU from step 2 can stay connected - it doesn't clash.
//
// WHAT YOU'LL SEE
//   Serial Monitor when it works:
//       STEP 3: one ToF sensor
//         ToF L  XSHUT=GPIO18  addr=0x30  PASS
//       1/1 sensors OK
//       ------------
//       L:142 mm          <- hand about 14cm in front of the sensor
//       L:87 mm           <- hand moved closer
//       L:---             <- nothing in front of it (normal, = no wall)
//   If it fails:
//         ToF L  XSHUT=GPIO18  addr=0x30  FAIL
//           -> check VIN->3V3, GND, SDA/SCL, and XSHUT->GPIO18
//       ...and every reading line shows L:FAIL
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
  Serial.println("\nSTEP 3: one ToF sensor");

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
