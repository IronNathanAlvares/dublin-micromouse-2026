// =====================================================================
// STEP 7 - One motor (Motor A): forward, stop, reverse, stop, repeat
// =====================================================================
// WIRING  (motor driver with ONE direction pin + ONE PWM pin per motor)
//   Driver DIR1  -> ESP32 GPIO0
//   Driver PWM1  -> ESP32 GPIO2
//   Driver GND   -> ESP32 GND  AND battery -   (all three joined!)
//   Driver VM    -> battery +
//   Driver VCC   -> ESP32 3V3   (only if your driver board has a VCC pin)
//   Motor's two power wires (M1/M2) -> driver's Motor A outputs
//   Encoder wires (C1, C2, encoder VCC/GND) are not needed for this test.
//
// WHAT YOU'LL SEE  (hold the wheel off the table)
//   LED and motor, repeating forever:
//       RED     1.5s  at startup only - motor off, get your hands clear
//       GREEN   2s    motor spins forward
//       RED     1s    stopped
//       BLUE    2s    motor spins backward
//       RED     1s    stopped
//   Serial Monitor (optional - the LED is enough):
//       FORWARD   speed=+140
//       stop      speed=+0
//       BACKWARD  speed=-140
//       stop      speed=+0
//   Spinning the "wrong" way on GREEN is fine - swap the two motor wires.
//
// IF THE LED CYCLES BUT THE MOTOR DOESN'T MOVE, THE CODE IS FINE. Check:
//   PWM1 wire actually connected  |  DIR1 wire connected
//   Driver GND joined to ESP32 GND  |  battery on VM and not flat
//   Driver STBY / EN / SLP pin (if it has one) tied HIGH to 3V3
//
// Speed is capped at 170/255 = about 6V average from a 9V battery, the
// N20 motors' rating. Don't raise SPEED_MAX.
// =====================================================================

#include <Arduino.h>

constexpr uint8_t DIR1_PIN = 0;
constexpr uint8_t PWM1_PIN = 2;

constexpr uint32_t PWM_FREQ = 20000;  // 20 kHz, above hearing
constexpr uint8_t  PWM_RES  = 8;      // duty 0..255

constexpr int16_t SPEED_MAX  = 170;
constexpr int16_t SPEED_TEST = 140;

#ifndef RGB_BUILTIN
#define RGB_BUILTIN 8
#endif

// Sign = direction, size = speed. Clamped so nothing can exceed SPEED_MAX.
void setMotor(int16_t speed) {
  speed = constrain(speed, -SPEED_MAX, SPEED_MAX);
  digitalWrite(DIR1_PIN, speed >= 0 ? HIGH : LOW);
  ledcWrite(PWM1_PIN, abs(speed));
}

void step(const char* label, int16_t speed, uint8_t r, uint8_t g, uint8_t b, uint32_t ms) {
  rgbLedWrite(RGB_BUILTIN, r, g, b);
  setMotor(speed);
  Serial.printf("%-9s speed=%+d\n", label, speed);
  delay(ms);
}

void setup() {
  Serial.begin(115200);

  pinMode(DIR1_PIN, OUTPUT);
  digitalWrite(DIR1_PIN, LOW);
  ledcAttach(PWM1_PIN, PWM_FREQ, PWM_RES);
  ledcWrite(PWM1_PIN, 0);  // motor off until we say so

  Serial.println("\nSTEP 7: one motor");
  rgbLedWrite(RGB_BUILTIN, 32, 0, 0);
  delay(1500);  // a moment to get your hands clear
}

void loop() {
  step("FORWARD",  SPEED_TEST, 0, 32, 0, 2000);
  step("stop",     0,          32, 0, 0, 1000);
  step("BACKWARD", -SPEED_TEST, 0, 0, 32, 2000);
  step("stop",     0,          32, 0, 0, 1000);
}
