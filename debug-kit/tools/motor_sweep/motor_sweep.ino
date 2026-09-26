// =====================================================================
// TOOL - Motor sweep: find out if the motor just needs more power
// =====================================================================
// WIRING  (same as step 7)
//   Driver DIR1 -> GPIO0     Driver PWM1 -> GPIO2
//   Driver GND  -> ESP32 GND AND battery -     Driver VM -> battery +
//
// WHAT YOU'LL SEE  (no Serial output - watch the LED and the motor)
//   BLUE   2s   at startup only - motor off
//   GREEN  ~6s  motor power rises slowly from 0 to max going FORWARD,
//               holds at max for 1.5s, then stops
//   RED    ~6s  the same, going BACKWARD
//   ...repeats forever.
//   A working motor starts turning partway through the ramp and speeds up.
//
//   What it tells you:
//       Starts partway up the ramp -> motor is fine, it just needs more
//                                     power than you were giving it
//       Hums but never turns       -> battery flat/sagging, or wheel or
//                                     gearbox jammed
//       Silent the whole time      -> no power reaching the motor: check
//                                     the wiring (see the README)
// =====================================================================

#include <Arduino.h>

constexpr uint8_t DIR1_PIN = 0;
constexpr uint8_t PWM1_PIN = 2;

constexpr uint32_t PWM_FREQ = 20000;
constexpr uint8_t  PWM_RES  = 8;

// 170/255 of the 9V rail ~= 6V average, the N20's rating. Never exceed.
constexpr uint8_t SPEED_MAX = 170;

#ifndef RGB_BUILTIN
#define RGB_BUILTIN 8
#endif

// Walk the duty up so you can see the exact point the shaft starts turning.
void sweep(bool forward) {
  digitalWrite(DIR1_PIN, forward ? HIGH : LOW);
  for (uint8_t duty = 0; duty <= SPEED_MAX; duty += 5) {
    ledcWrite(PWM1_PIN, duty);
    delay(120);
  }
  ledcWrite(PWM1_PIN, SPEED_MAX);
  delay(1500);
  ledcWrite(PWM1_PIN, 0);
}

void setup() {
  pinMode(DIR1_PIN, OUTPUT);
  digitalWrite(DIR1_PIN, LOW);

  ledcAttach(PWM1_PIN, PWM_FREQ, PWM_RES);
  ledcWrite(PWM1_PIN, 0);

  rgbLedWrite(RGB_BUILTIN, 0, 0, 32);  // blue: alive, hands clear
  delay(2000);
}

void loop() {
  rgbLedWrite(RGB_BUILTIN, 0, 32, 0);
  sweep(true);
  delay(1000);

  rgbLedWrite(RGB_BUILTIN, 32, 0, 0);
  sweep(false);
  delay(1000);
}
