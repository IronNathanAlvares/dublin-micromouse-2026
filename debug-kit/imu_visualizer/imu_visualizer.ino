// =====================================================================
// IMU VISUALIZER (sketch half) - streams the IMU to the web page
// =====================================================================
// This sketch goes with index.html in this same folder. It reads the
// MPU-6050 and sends its readings over USB in a format the web page
// understands, so the page can draw a 3D board that follows yours.
//
// WIRING  (same as step 2 - nothing new)
//   MPU VCC -> ESP32 3V3
//   MPU GND -> ESP32 GND
//   MPU SDA -> ESP32 GPIO6
//   MPU SCL -> ESP32 GPIO7
//   Leave XDA, XCL, AD0 and INT UNCONNECTED.
//
// HOW TO USE IT
//   1. Upload this sketch (Tools -> USB CDC On Boot: Enabled).
//   2. CLOSE the Arduino Serial Monitor. Only one program can use the
//      USB port at a time, so the page can't connect while it's open.
//   3. Open index.html in Chrome or Edge (double-click it).
//   4. Click "Connect board" and pick the ESP32's COM port.
//   5. Keep the board STILL for one second (it calibrates), then turn it.
//
// WHAT YOU'LL SEE  (if you open the Serial Monitor instead of the page)
//       # imu_visualizer
//       # idle SDA=1 SCL=1
//       # PASS: MPU at 0x68, chip ID 0x68
//       # calibrating gyro - keep still
//       # gyro bias -0.41 dps
//       IMU,0.00,0.12,1.80,-0.60      <- 25 of these a second
//   Lines starting with "#" are messages for people. Lines starting with
//   "IMU," are data for the page: heading, turn rate, pitch, roll.
//     heading  degrees turned since start, counter-clockwise from above
//     turn rate  how fast it's turning now, in degrees per second
//     pitch    nose up (+) / nose down (-), from the accelerometer
//     roll     tilt left / right, from the accelerometer
//
// COMMANDS THE PAGE CAN SEND
//   z  set heading back to 0        c  re-calibrate the gyro (keep still)
//
// Works with clone MPU chips. If SDA/SCL are swapped it finds them anyway
// and says so.
// =====================================================================

#include <Arduino.h>
#include <Wire.h>

constexpr uint8_t PIN_SDA = 6;
constexpr uint8_t PIN_SCL = 7;

constexpr uint8_t REG_CONFIG       = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG  = 0x1B;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t REG_GYRO_ZOUT_H  = 0x47;
constexpr uint8_t REG_PWR_MGMT_1   = 0x6B;
constexpr uint8_t REG_WHO_AM_I     = 0x75;

constexpr float    GYRO_LSB_PER_DPS = 65.5f;     // at +/-500 dps
constexpr float    ACCEL_LSB_PER_G  = 16384.0f;  // at the default +/-2 g
constexpr uint16_t CAL_SAMPLES      = 500;
constexpr uint32_t SEND_PERIOD_MS   = 40;        // 25 updates a second
constexpr float    TILT_SMOOTHING   = 0.2f;      // 1 = raw, lower = smoother

uint8_t  mpuAddr    = 0;
float    gyroZBias  = 0.0f;
float    heading    = 0.0f;
float    pitchDeg   = 0.0f;
float    rollDeg    = 0.0f;
bool     tiltReady  = false;
uint32_t lastMicros = 0;
uint32_t lastSendMs = 0;

// Scans the whole bus on the given pins; returns 0x68/0x69 if an MPU answered.
uint8_t scanBus(uint8_t sda, uint8_t scl) {
  Wire.end();
  Wire.begin(sda, scl);
  Wire.setClock(100000);

  Serial.printf("# Scan SDA=GPIO%u SCL=GPIO%u:", sda, scl);
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

bool readBytes(uint8_t reg, uint8_t* buf, uint8_t n) {
  Wire.beginTransmission(mpuAddr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(mpuAddr, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

int16_t be16(const uint8_t* p) {
  return (int16_t)((p[0] << 8) | p[1]);
}

// Returns false (and leaves 'out' alone) if the read failed.
bool readGyroZdps(float& out) {
  uint8_t b[2];
  if (!readBytes(REG_GYRO_ZOUT_H, b, 2)) return false;
  out = be16(b) / GYRO_LSB_PER_DPS;
  return true;
}

void updateTilt() {
  uint8_t b[6];
  if (!readBytes(REG_ACCEL_XOUT_H, b, 6)) return;
  const float ax = be16(b)     / ACCEL_LSB_PER_G;
  const float ay = be16(b + 2) / ACCEL_LSB_PER_G;
  const float az = be16(b + 4) / ACCEL_LSB_PER_G;

  const float p = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
  const float r = atan2f(ay, az) * RAD_TO_DEG;
  if (!tiltReady) {
    pitchDeg = p;
    rollDeg  = r;
    tiltReady = true;
  } else {
    pitchDeg += TILT_SMOOTHING * (p - pitchDeg);
    rollDeg  += TILT_SMOOTHING * (r - rollDeg);
  }
}

void calibrateGyro() {
  Serial.println("# calibrating gyro - keep still");
  double sum = 0;
  uint16_t got = 0;
  for (uint16_t i = 0; i < CAL_SAMPLES; i++) {
    float dps;
    if (readGyroZdps(dps)) {
      sum += dps;
      got++;
    }
    delay(2);
  }
  if (got) gyroZBias = sum / got;
  heading = 0.0f;
  lastMicros = micros();
  Serial.printf("# gyro bias %.2f dps\n", gyroZBias);
}

void handleCommands() {
  while (Serial.available()) {
    const char c = Serial.read();
    if (c == 'z') {
      heading = 0.0f;
      Serial.println("# heading zeroed");
    } else if (c == 'c') {
      calibrateGyro();
    }
  }
}

void setup() {
  Serial.begin(115200);
  const uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 2000) {}
  Serial.println("\n# imu_visualizer");

  pinMode(PIN_SDA, INPUT);
  pinMode(PIN_SCL, INPUT);
  const int sdaIdle = digitalRead(PIN_SDA), sclIdle = digitalRead(PIN_SCL);
  Serial.printf("# idle SDA=%d SCL=%d\n", sdaIdle, sclIdle);
  if (!sdaIdle || !sclIdle) {
    Serial.println("# idle should be 1 - a wire isn't connected, or the IMU has no power");
  }

  mpuAddr = scanBus(PIN_SDA, PIN_SCL);
  if (!mpuAddr) {
    mpuAddr = scanBus(PIN_SCL, PIN_SDA);
    if (mpuAddr) Serial.println("# SDA and SCL are SWAPPED - swap the two wires. Running anyway.");
  }
  if (!mpuAddr) {
    // Repeat forever so the page shows it even if it connects late.
    while (true) {
      Serial.println("# FAIL: no MPU found. Check VCC/GND, use SDA/SCL (not XDA/XCL), leave INT unconnected.");
      delay(2000);
    }
  }

  uint8_t id = 0;
  readBytes(REG_WHO_AM_I, &id, 1);
  Serial.printf("# PASS: MPU at 0x%02X, chip ID 0x%02X%s\n", mpuAddr, id,
                id == 0x68 ? "" : " (clone - fine)");

  writeReg(REG_PWR_MGMT_1, 0x01);   // wake up
  delay(100);
  writeReg(REG_GYRO_CONFIG, 0x08);  // +/-500 dps
  writeReg(REG_CONFIG, 0x04);       // ~21 Hz low-pass

  calibrateGyro();
}

void loop() {
  handleCommands();

  // Integrate the gyro every ~4 ms so quick turns aren't missed,
  // but only send to the page 25 times a second.
  float gz;
  if (readGyroZdps(gz)) {
    gz -= gyroZBias;
    const uint32_t now = micros();
    heading += gz * (now - lastMicros) * 1e-6f;
    lastMicros = now;

    if (millis() - lastSendMs >= SEND_PERIOD_MS) {
      lastSendMs = millis();
      updateTilt();
      Serial.printf("IMU,%.2f,%.2f,%.2f,%.2f\n", heading, gz, pitchDeg, rollDeg);
    }
  }
  delay(4);
}
