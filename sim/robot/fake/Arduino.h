// Arduino.h (fake) - just enough of the ESP32 Arduino core for the firmware
// to compile and run on a laptop against the virtual robot.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "SimHardware.h"

using std::max;
using std::min;

#define HIGH 1
#define LOW 0
#define INPUT 0x01
#define OUTPUT 0x03
#define INPUT_PULLUP 0x05
#define RISING 0x01
#define FALLING 0x02
#define CHANGE 0x03
#define IRAM_ATTR

// Every clock read costs a microsecond, so busy-wait loops always make progress.
inline uint32_t millis() { simhw::advanceUs(1); return uint32_t(simhw::nowUs() / 1000); }
inline uint32_t micros() { simhw::advanceUs(1); return uint32_t(simhw::nowUs()); }
inline void delay(uint32_t ms) { simhw::advanceUs(uint64_t(ms) * 1000); }
inline void delayMicroseconds(uint32_t us) { simhw::advanceUs(us); }

inline void pinMode(uint8_t, uint8_t) {}
inline void digitalWrite(uint8_t pin, uint8_t level) { simhw::pinWrite(pin, level); }
inline int digitalRead(uint8_t pin) { return simhw::pinRead(pin); }
inline bool ledcAttach(uint8_t, uint32_t, uint8_t) { return true; }
inline bool ledcWrite(uint8_t pin, uint32_t duty) { simhw::pwmWrite(pin, duty); return true; }
inline void rgbLedWrite(uint8_t, uint8_t r, uint8_t g, uint8_t b) { simhw::led(r, g, b); }
inline int digitalPinToInterrupt(int pin) { return pin; }
inline void attachInterrupt(int pin, void (*isr)(), int) { simhw::attachIsr(uint8_t(pin), isr); }
inline void noInterrupts() {}
inline void interrupts() {}

template <class T, class L, class H>
T constrain(T x, L lo, H hi) { return x < T(lo) ? T(lo) : x > T(hi) ? T(hi) : x; }

class HardwareSerial {
 public:
  void begin(unsigned long) {}
  explicit operator bool() const { return true; }
  int available() { return simhw::serialAvailable(); }
  int read() { return simhw::serialRead(); }
  void print(const char* s) { simhw::serialWrite(s); }
  void print(char c) { const char s[2] = {c, 0}; simhw::serialWrite(s); }
  void print(int v) { printf("%d", v); }
  void println() { simhw::serialWrite("\n"); }
  void println(const char* s) { print(s); println(); }
  void println(char c) { print(c); println(); }
  int printf(const char* fmt, ...) __attribute__((format(printf, 2, 3))) {
    char buf[512];
    va_list args;
    va_start(args, fmt);
    const int n = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    simhw::serialWrite(buf);
    return n;
  }
};
inline HardwareSerial Serial;
