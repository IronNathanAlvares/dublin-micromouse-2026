// =====================================================================
// STEP 2 - IMU (MPU-6050 / GY-521) on its own
// =====================================================================
// WIRING (4 wires, nothing else on the I2C pins yet)
//   MPU VCC -> ESP32 3V3
//   MPU GND -> ESP32 GND
//   MPU SDA -> ESP32 GPIO6
//   MPU SCL -> ESP32 GPIO7
//   Leave XDA, XCL, AD0 and INT UNCONNECTED.
//
// WHAT YOU'LL SEE  (keep the board still for the first second)
//   Serial Monitor when it works:
//       STEP 2: IMU test
//       idle SDA=1 SCL=1 (both should be 1)
//       Scan SDA=GPIO6 SCL=GPIO7: 0x68
//       PASS: MPU at 0x68, chip ID 0x68 (genuine)
//       Calibrating - keep STILL...
//       done (bias -0.41 dps). Now rotate the board.
//       heading:   0.0 deg   gyroZ:   0.1 dps     <- sitting still
//       heading:  47.3 deg   gyroZ:  92.8 dps     <- while you turn it
//       heading:  90.4 deg   gyroZ:   0.2 dps     <- stopped after a 1/4 turn
//   Good means: gyroZ is near 0 when still, and heading goes to about +90
//   for a quarter turn one way and -90 the other. A slow creep of the
//   heading while still is normal.
//
//   Other things you might see:
//       chip ID 0x70 (clone - fine)              -> clone chip, works fine
//       !! SDA and SCL are SWAPPED ...           -> swap those two wires
//       note: MPU at 0x69 because AD0 is HIGH    -> disconnect AD0
//       idle SDA=0 SCL=0                         -> wire not connected, or
//                                                   the IMU has no power
//       FAIL: no MPU found either way round.     -> check VCC/GND, use SDA/SCL
//                                                   not XDA/XCL, INT unplugged
//
// No libraries needed. Works with clone MPU chips too.
// If SDA/SCL are swapped, or the chip is at 0x69, this sketch finds it
// anyway and tells you what to fix.
// =====================================================================

#include <Arduino.h>
#include <Wire.h>

constexpr uint8_t PIN_SDA = 6;
constexpr uint8_t PIN_SCL = 7;

constexpr uint8_t REG_CONFIG      = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
constexpr uint8_t REG_GYRO_ZOUT_H = 0x47;
constexpr uint8_t REG_PWR_MGMT_1  = 0x6B;
constexpr uint8_t REG_WHO_AM_I    = 0x75;

constexpr float    GYRO_LSB_PER_DPS = 65.5f;  // at +/-500 dps
constexpr uint16_t CAL_SAMPLES      = 500;

uint8_t  mpuAddr    = 0;
float    gyroZBias  = 0.0f;
float    heading    = 0.0f;
uint32_t lastMicros = 0;

// Scans the whole bus on the given pins; returns 0x68/0x69 if an MPU answered.
uint8_t scanBus(uint8_t sda, uint8_t scl) {
  Wire.end();
  Wire.begin(sda, scl);
  Wire.setClock(100000);

  Serial.printf("Scan SDA=GPIO%u SCL=GPIO%u:", sda, scl);
  uint8_t mpu = 0, count = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf(" 0x%02X", addr);
      count++;
      if (addr == 0x68 || addr == 0x69) mpu = addr;
    }
  }
  Serial.println(count ? "" : " nothing");
  return mpu;
}

void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(mpuAddr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

int readReg(uint8_t reg) {
  Wire.beginTransmission(mpuAddr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return -1;
  if (Wire.requestFrom(mpuAddr, (uint8_t)1) != 1) return -1;
  return Wire.read();
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

void setup() {
  Serial.begin(115200);
  const uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 2000) {}
  Serial.println("\nSTEP 2: IMU test");

  pinMode(PIN_SDA, INPUT);
  pinMode(PIN_SCL, INPUT);
  const int sdaIdle = digitalRead(PIN_SDA), sclIdle = digitalRead(PIN_SCL);
  Serial.printf("idle SDA=%d SCL=%d (both should be 1)\n", sdaIdle, sclIdle);
  if (!sdaIdle || !sclIdle) {
    Serial.println("  0 = wire not connected, or MPU has no power (check VCC/GND)");
  }

  mpuAddr = scanBus(PIN_SDA, PIN_SCL);
  if (!mpuAddr) {
    mpuAddr = scanBus(PIN_SCL, PIN_SDA);
    if (mpuAddr) Serial.println("!! SDA and SCL are SWAPPED - swap the two wires. Running anyway.");
  }
  if (!mpuAddr) {
    Serial.println("FAIL: no MPU found either way round.");
    Serial.println("  Check VCC/GND, use SDA/SCL (not XDA/XCL), leave INT unconnected.");
    while (true) delay(1000);
  }
  if (mpuAddr == 0x69) Serial.println("note: MPU at 0x69 because AD0 is HIGH - disconnect AD0.");

  const int id = readReg(REG_WHO_AM_I);
  Serial.printf("PASS: MPU at 0x%02X, chip ID 0x%02X%s\n", mpuAddr, id,
                id == 0x68 ? " (genuine)" : " (clone - fine)");

  writeReg(REG_PWR_MGMT_1, 0x01);   // wake up
  delay(100);
  writeReg(REG_GYRO_CONFIG, 0x08);  // +/-500 dps
  writeReg(REG_CONFIG, 0x04);       // ~21 Hz low-pass

  Serial.println("Calibrating - keep STILL...");
  double sum = 0;
  for (uint16_t i = 0; i < CAL_SAMPLES; i++) {
    sum += readGyroZdps();
    delay(2);
  }
  gyroZBias = sum / CAL_SAMPLES;
  Serial.printf("done (bias %.2f dps). Now rotate the board.\n", gyroZBias);
  lastMicros = micros();
}

void loop() {
  const float    gz  = readGyroZdps() - gyroZBias;
  const uint32_t now = micros();
  heading += gz * (now - lastMicros) * 1e-6f;
  lastMicros = now;

  Serial.printf("heading:%6.1f deg   gyroZ:%6.1f dps\n", heading, gz);
  delay(100);
}
