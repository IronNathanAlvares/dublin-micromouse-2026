// =====================================================================
// STEP 6 - All three ToF sensors + IMU together
// =====================================================================
// WIRING  (= step 2 + step 5 combined, nothing new to connect)
//   Shared by all 4 boards:
//     VIN/VCC -> ESP32 3V3        GND -> ESP32 GND
//     SDA     -> ESP32 GPIO6      SCL -> ESP32 GPIO7
//   ToF XSHUT wires:
//     LEFT  -> GPIO18    FRONT -> GPIO19    RIGHT -> GPIO20
//   IMU: leave XDA, XCL, AD0 and INT unconnected.
//
// WHAT YOU'LL SEE  (keep the robot still for the first second)
//   Serial Monitor when it works:
//       STEP 6: 3x ToF + IMU
//         ToF L  XSHUT=GPIO18  addr=0x30  PASS
//         ToF F  XSHUT=GPIO19  addr=0x31  PASS
//         ToF R  XSHUT=GPIO20  addr=0x29  PASS
//         IMU        addr=0x68  PASS
//       Calibrating gyro - keep STILL...
//       ------------
//       L:142 mm  F:310 mm  R:98 mm   heading:0.0deg  gyroZ:0.1dps
//       L:140 mm  F:---     R:99 mm   heading:24.6deg  gyroZ:61.2dps
//   Distances change when you wave a hand in front of each sensor, and the
//   heading changes when you rotate the robot (about 90 per quarter turn).
//   A device that fails shows FAIL at startup and FAIL in its column.
//
// If a device FAILs here but passed in its own step, the last thing you
// connected is the problem - unplug it and re-run the earlier step.
//
// LIBRARY: Library Manager -> "VL53L0X" by Pololu (IMU needs no library)
// =====================================================================

#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>

constexpr uint8_t PIN_SDA = 6;
constexpr uint8_t PIN_SCL = 7;

// ---------- ToF ----------
constexpr uint16_t MAX_VALID_MM = 2000;  // beyond this = no wall

struct Tof {
  const char* name;
  uint8_t     xshut;
  uint8_t     addr;
  VL53L0X     sensor;
  bool        ok;
};

Tof tofs[] = {
  {"L", 18, 0x30, VL53L0X(), false},
  {"F", 19, 0x31, VL53L0X(), false},
  {"R", 20, 0x29, VL53L0X(), false},
};

bool startTof(Tof& t) {
  digitalWrite(t.xshut, HIGH);
  delay(10);
  t.sensor.setTimeout(100);
  if (!t.sensor.init()) {
    digitalWrite(t.xshut, LOW);  // don't let a failed sensor squat on 0x29
    return false;
  }
  if (t.addr != 0x29) t.sensor.setAddress(t.addr);
  t.sensor.startContinuous();
  return true;
}

// ---------- IMU (MPU-6050, raw registers) ----------
constexpr uint8_t  REG_CONFIG       = 0x1A;
constexpr uint8_t  REG_GYRO_CONFIG  = 0x1B;
constexpr uint8_t  REG_GYRO_ZOUT_H  = 0x47;
constexpr uint8_t  REG_PWR_MGMT_1   = 0x6B;
constexpr float    GYRO_LSB_PER_DPS = 65.5f;  // at +/-500 dps
constexpr uint16_t CAL_SAMPLES      = 500;

uint8_t  mpuAddr    = 0;  // 0 = not found
float    gyroZBias  = 0.0f;
float    heading    = 0.0f;
uint32_t lastMicros = 0;

bool present(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(mpuAddr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

float readGyroZdps() {
  Wire.beginTransmission(mpuAddr);
  Wire.write(REG_GYRO_ZOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(mpuAddr, (uint8_t)2);
  const uint8_t hi = Wire.read();  // two separate reads: C++ doesn't fix the
  const uint8_t lo = Wire.read();  // order of two reads inside one expression
  const int16_t raw = (int16_t)((hi << 8) | lo);
  return raw / GYRO_LSB_PER_DPS;
}

bool startImu() {
  if (present(0x68))      mpuAddr = 0x68;
  else if (present(0x69)) mpuAddr = 0x69;
  else return false;

  mpuWrite(REG_PWR_MGMT_1, 0x01);   // wake up
  delay(100);
  mpuWrite(REG_GYRO_CONFIG, 0x08);  // +/-500 dps
  mpuWrite(REG_CONFIG, 0x04);       // ~21 Hz low-pass, tames motor vibration
  return true;
}

// ---------- main ----------
void setup() {
  Serial.begin(115200);
  const uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 2000) {}
  Serial.println("\nSTEP 6: 3x ToF + IMU");

  for (auto& t : tofs) {
    pinMode(t.xshut, OUTPUT);
    digitalWrite(t.xshut, LOW);
  }
  delay(10);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);

  for (auto& t : tofs) {
    t.ok = startTof(t);
    Serial.printf("  ToF %s  XSHUT=GPIO%u  addr=0x%02X  %s\n",
                  t.name, t.xshut, t.addr, t.ok ? "PASS" : "FAIL");
  }

  const bool imuOk = startImu();
  if (imuOk) Serial.printf("  IMU        addr=0x%02X  PASS\n", mpuAddr);
  else       Serial.println("  IMU        addr=0x68  FAIL -> re-run step 2 (02_imu)");

  if (imuOk) {
    Serial.println("Calibrating gyro - keep STILL...");
    double sum = 0;
    for (uint16_t i = 0; i < CAL_SAMPLES; i++) {
      sum += readGyroZdps();
      delay(2);
    }
    gyroZBias = sum / CAL_SAMPLES;
  }
  Serial.println("------------");
  lastMicros = micros();
}

void loop() {
  for (auto& t : tofs) {
    Serial.printf("%s:", t.name);
    if (!t.ok) { Serial.print("FAIL    "); continue; }
    const uint16_t mm = t.sensor.readRangeContinuousMillimeters();
    if (t.sensor.timeoutOccurred() || mm >= MAX_VALID_MM) Serial.print("---     ");
    else Serial.printf("%-4umm  ", mm);
  }

  if (mpuAddr) {
    const float    gz  = readGyroZdps() - gyroZBias;
    const uint32_t now = micros();
    heading += gz * (now - lastMicros) * 1e-6f;
    lastMicros = now;
    Serial.printf(" heading:%.1fdeg  gyroZ:%.1fdps\n", heading, gz);
  } else {
    Serial.println(" heading:FAIL");
  }
  delay(100);  // ~10 Hz
}
