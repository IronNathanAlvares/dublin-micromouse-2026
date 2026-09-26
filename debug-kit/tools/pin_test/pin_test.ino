// =====================================================================
// TOOL - Pin test: are GPIO0 and GPIO2 actually switching?
// =====================================================================
// !! UNPLUG THE MOTOR FROM THE DRIVER FIRST. This sets PWM fully on,
// !! which puts the whole 9V across a 6V motor.
//
// WIRING
//   Nothing extra. Measure GPIO0 and GPIO2 with a multimeter set to DC
//   volts, black probe on GND. Each should read 0V or 3.3V to match the
//   LED stage.
//
// WHAT YOU'LL SEE  (no Serial output - watch the LED, measure with a meter)
//   Multimeter on DC volts, black probe on GND, red probe on the pin:
//       LED        GPIO0    GPIO2
//       BLUE       -        -        2s at startup only
//       GREEN      3.3V     3.3V     2s
//       YELLOW     3.3V     0V       2s
//       RED        0V       0V       2s
//   ...repeats from GREEN.
//
//   What it tells you:
//       Readings match the table -> ESP32 pins are fine; the problem is the
//                                   driver, its power, or the wires to it
//       A pin stuck at 0V        -> damaged pin, or shorted to GND
//       No BLUE at startup       -> this sketch didn't upload; the old one
//                                   is still running
// =====================================================================

#include <Arduino.h>

constexpr uint8_t DIR1_PIN = 0;
constexpr uint8_t PWM1_PIN = 2;

#ifndef RGB_BUILTIN
#define RGB_BUILTIN 8
#endif

void setup() {
  pinMode(DIR1_PIN, OUTPUT);
  pinMode(PWM1_PIN, OUTPUT);   // plain output, deliberately not ledcAttach

  rgbLedWrite(RGB_BUILTIN, 0, 0, 32);
  delay(2000);
}

void loop() {
  digitalWrite(DIR1_PIN, HIGH);
  digitalWrite(PWM1_PIN, HIGH);
  rgbLedWrite(RGB_BUILTIN, 0, 32, 0);   // green = both pins high
  delay(2000);

  digitalWrite(PWM1_PIN, LOW);
  rgbLedWrite(RGB_BUILTIN, 32, 32, 0);  // yellow = DIR high, PWM low
  delay(2000);

  digitalWrite(DIR1_PIN, LOW);
  rgbLedWrite(RGB_BUILTIN, 32, 0, 0);   // red = both pins low
  delay(2000);
}
