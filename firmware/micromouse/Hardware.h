// Hardware.h - motors, encoders, distance sensors, IMU and LED.
// Each part mirrors the matching Debug Kit sketch, so if something misbehaves
// here, re-run that Debug Kit step first. Included once, from micromouse.ino.
#pragma once

#include <Arduino.h>
#include <VL53L0X.h>  // Library Manager: "VL53L0X" by Pololu (NOT Adafruit)
#include <Wire.h>

#include "config.h"

// ================================ LED ===================================
inline void led(uint8_t r, uint8_t g, uint8_t b) { rgbLedWrite(PIN_RGB, r, g, b); }

inline bool buttonDown() { return digitalRead(PIN_BUTTON) == LOW; }

// ============================== Motors ==================================
// Debug Kit step 8. Positive = forward.
namespace motors {

inline void writeOne(uint8_t dirPin, uint8_t pwmPin, int pwm, bool reversed) {
  pwm = constrain(pwm, -PWM_LIMIT, PWM_LIMIT);
  if (reversed) pwm = -pwm;
  digitalWrite(dirPin, pwm >= 0 ? HIGH : LOW);
  ledcWrite(pwmPin, abs(pwm));
}

inline void set(int left, int right) {
  writeOne(PIN_DIR_LEFT, PIN_PWM_LEFT, left, MOTOR_LEFT_REVERSED);
  writeOne(PIN_DIR_RIGHT, PIN_PWM_RIGHT, right, MOTOR_RIGHT_REVERSED);
}

inline void stop() { set(0, 0); }

inline void begin() {
  pinMode(PIN_DIR_LEFT, OUTPUT);
  pinMode(PIN_DIR_RIGHT, OUTPUT);
  ledcAttach(PIN_PWM_LEFT, PWM_FREQ_HZ, PWM_BITS);
  ledcAttach(PIN_PWM_RIGHT, PWM_FREQ_HZ, PWM_BITS);
  stop();
}

}  // namespace motors

// ============================= Encoders =================================
// Debug Kit encoder tool, but counting both edges of A for twice the detail.
namespace encoders {

volatile int32_t leftCount = 0;
volatile int32_t rightCount = 0;

void IRAM_ATTR onLeft() {
  leftCount = leftCount + ((digitalRead(PIN_ENC_LEFT_A) == digitalRead(PIN_ENC_LEFT_B)) ? 1 : -1);
}
void IRAM_ATTR onRight() {
  rightCount = rightCount + ((digitalRead(PIN_ENC_RIGHT_A) == digitalRead(PIN_ENC_RIGHT_B)) ? 1 : -1);
}

inline int32_t left() { return ENC_LEFT_REVERSED ? -leftCount : leftCount; }
inline int32_t right() { return ENC_RIGHT_REVERSED ? -rightCount : rightCount; }

inline void zero() {
  noInterrupts();
  leftCount = 0;
  rightCount = 0;
  interrupts();
}

inline void begin() {
  // Hall encoders are open-drain and need pull-ups for clean edges.
  pinMode(PIN_ENC_LEFT_A, INPUT_PULLUP);
  pinMode(PIN_ENC_LEFT_B, INPUT_PULLUP);
  pinMode(PIN_ENC_RIGHT_A, INPUT_PULLUP);
  pinMode(PIN_ENC_RIGHT_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_LEFT_A), onLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_RIGHT_A), onRight, CHANGE);
}

}  // namespace encoders

// ========================= Distance sensors =============================
// Debug Kit steps 3-6, made non-blocking so the motor loop never waits on I2C.
namespace tof {

enum Side { LEFT = 0, FRONT = 1, RIGHT = 2 };
constexpr uint16_t NONE = 0xFFFF;  // no reading / nothing in range

struct Sensor {
  const char* name;
  uint8_t xshut;
  uint8_t address;
  VL53L0X dev;
  bool ok;
  uint16_t mm;
  uint32_t readAtMs;
};

Sensor sensors[3] = {
    {"L", PIN_XSHUT_LEFT, TOF_ADDR_LEFT, VL53L0X(), false, NONE, 0},
    {"F", PIN_XSHUT_FRONT, TOF_ADDR_FRONT, VL53L0X(), false, NONE, 0},
    {"R", PIN_XSHUT_RIGHT, TOF_ADDR_RIGHT, VL53L0X(), false, NONE, 0},
};

// Wakes the sensors one at a time and gives each its own address.
// Call after Wire.begin(). Returns true if all three answered.
inline bool begin() {
  for (auto& s : sensors) {
    pinMode(s.xshut, OUTPUT);
    digitalWrite(s.xshut, LOW);
  }
  delay(10);
  bool all = true;
  for (auto& s : sensors) {
    digitalWrite(s.xshut, HIGH);
    delay(10);
    s.dev.setTimeout(100);
    s.ok = s.dev.init();
    if (!s.ok) {
      digitalWrite(s.xshut, LOW);  // don't let a failed sensor squat on 0x29
      all = false;
      continue;
    }
    if (s.address != 0x29) s.dev.setAddress(s.address);
    s.dev.setMeasurementTimingBudget(20000);  // 20 ms per reading (default is 33)
    s.dev.startContinuous();
  }
  return all;
}

// Collects any finished measurements. Cheap: call it as often as you like.
inline void update() {
  for (auto& s : sensors) {
    if (!s.ok) continue;
    if ((s.dev.readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) == 0) continue;  // not ready
    const uint16_t mm = s.dev.readRangeContinuousMillimeters();
    s.mm = (s.dev.timeoutOccurred() || mm >= TOF_MAX_MM) ? NONE : mm;
    s.readAtMs = millis();
  }
}

// Latest distance in mm, or NONE if there's nothing in range or no recent reading.
inline uint16_t read(Side side) {
  const Sensor& s = sensors[side];
  if (!s.ok || millis() - s.readAtMs > 150) return NONE;
  return s.mm;
}

// Blocks until every sensor has a reading taken after this call (max ~100 ms),
// so wall decisions use what's in front of the mouse NOW.
inline void waitFresh() {
  const uint32_t since = millis();
  while (millis() - since < 100) {
    update();
    bool fresh = true;
    for (auto& s : sensors) {
      if (s.ok && int32_t(s.readAtMs - since) < 1) fresh = false;
    }
    if (fresh) return;
    delay(1);
  }
}

}  // namespace tof

// ================================ IMU ===================================
// MPU-6050 by raw registers, as in Debug Kit steps 2 and 6. Only yaw is used.
namespace imu {

constexpr uint8_t REG_CONFIG = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
constexpr uint8_t REG_GYRO_ZOUT_H = 0x47;
constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
constexpr float LSB_PER_DPS = 32.8f;  // at +/-1000 dps: spins can pass 500 dps

uint8_t address = 0;  // 0 = not found
float biasDps = 0;
float yawDeg = 0;     // anticlockwise positive, integrated from the gyro
float rateDps = 0;
float rawDps = 0;     // last reading before bias correction
uint32_t lastMicros = 0;

inline bool present(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

inline void writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

inline float readRawDps() {
  Wire.beginTransmission(address);
  Wire.write(REG_GYRO_ZOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(address, (uint8_t)2);
  const uint8_t hi = Wire.read();
  const uint8_t lo = Wire.read();
  return GYRO_Z_SIGN * int16_t((hi << 8) | lo) / LSB_PER_DPS;
}

inline bool begin() {
  if (present(0x68)) address = 0x68;
  else if (present(0x69)) address = 0x69;
  else return false;
  writeReg(REG_PWR_MGMT_1, 0x01);  // wake up, gyro clock
  delay(100);
  writeReg(REG_GYRO_CONFIG, 0x10);  // +/-1000 dps
  writeReg(REG_CONFIG, 0x03);       // ~44 Hz low-pass: tames motor vibration, little lag
  lastMicros = micros();
  return true;
}

// The mouse must be completely still for this (~0.5 s).
inline void calibrate() {
  if (!address) return;
  double sum = 0;
  constexpr int SAMPLES = 250;
  for (int i = 0; i < SAMPLES; i++) {
    sum += readRawDps();
    delay(2);
  }
  biasDps = sum / SAMPLES;
  rateDps = 0;
  lastMicros = micros();
}

inline void update() {
  if (!address) return;
  rawDps = readRawDps();
  const float rate = (rawDps - biasDps) * GYRO_SCALE;
  const uint32_t now = micros();
  // Trapezoid rule: average of this reading and the last one. Using only the
  // newest reading loses ~0.5 degrees per fast turn.
  yawDeg += 0.5f * (rate + rateDps) * (now - lastMicros) * 1e-6f;
  rateDps = rate;
  lastMicros = now;
}

// Call only while the mouse is certainly not turning: then any reading is
// pure bias, so nudge the bias estimate towards it. Stops slow gyro drift.
inline void learnBias() {
  if (!address) return;
  // A bigger reading means it's actually still turning a little (e.g. just
  // after a spin): drift changes far more slowly than that.
  if (fabsf(rawDps - biasDps) > GYRO_BIAS_LEARN_MAX_DPS) return;
  biasDps += GYRO_BIAS_LEARN_GAIN * (rawDps - biasDps);
}

}  // namespace imu
