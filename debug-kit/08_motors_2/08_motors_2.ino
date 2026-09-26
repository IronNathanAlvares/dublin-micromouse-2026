// =====================================================================
// STEP 8 - Both motors: forward, backward, spin left, spin right
// =====================================================================
// WIRING  (= step 7 + Motor B)
//   Driver DIR1 -> ESP32 GPIO0     (already wired in step 7)
//   Driver PWM1 -> ESP32 GPIO2     (already wired in step 7)
//   Driver DIR2 -> ESP32 GPIO3     (new)
//   Driver PWM2 -> ESP32 GPIO10    (new - NOT GPIO5, that's a C6 strapping pin)
//   Driver GND  -> ESP32 GND  AND battery -
//   Driver VM   -> battery +
//   Driver VCC  -> ESP32 3V3   (only if your driver board has a VCC pin)
//   Left motor  -> driver Motor A outputs
//   Right motor -> driver Motor B outputs
//
// WHAT YOU'LL SEE  (hold the robot up, wheels in the air)
//   LED and wheels, repeating forever (RED 1s stop between each):
//       GREEN   2s   both wheels forward
//       BLUE    2s   both wheels backward
//       YELLOW  2s   spin LEFT:  left wheel back, right wheel forward
//       PURPLE  2s   spin RIGHT: left wheel forward, right wheel back
//   Serial Monitor (optional - the LED is enough):
//       FORWARD     left=+140 right=+140
//       stop        left=  +0 right=  +0
//       BACKWARD    left=-140 right=-140
//       stop        left=  +0 right=  +0
//       SPIN LEFT   left=-140 right=+140
//       stop        left=  +0 right=  +0
//       SPIN RIGHT  left=+140 right=-140
//       stop        left=  +0 right=  +0
//
// A wheel going the wrong way? Swap THAT motor's two wires.
// Left and right mixed up? Swap which motor goes to Motor A / Motor B.
// Motor A works but B doesn't? Check the DIR2 and PWM2 wires.
//
// Speed is capped at 170/255 = about 6V average from a 9V battery.
// =====================================================================

#include <Arduino.h>

constexpr uint8_t DIR1_PIN = 0;   // Motor A = left
constexpr uint8_t PWM1_PIN = 2;
constexpr uint8_t DIR2_PIN = 3;   // Motor B = right
constexpr uint8_t PWM2_PIN = 10;

constexpr uint32_t PWM_FREQ = 20000;
constexpr uint8_t  PWM_RES  = 8;

constexpr int16_t SPEED_MAX  = 170;
constexpr int16_t SPEED_TEST = 140;

#ifndef RGB_BUILTIN
#define RGB_BUILTIN 8
#endif

void setMotor(uint8_t dirPin, uint8_t pwmPin, int16_t speed) {
  speed = constrain(speed, -SPEED_MAX, SPEED_MAX);
  digitalWrite(dirPin, speed >= 0 ? HIGH : LOW);
  ledcWrite(pwmPin, abs(speed));
}

void drive(const char* label, int16_t left, int16_t right,
           uint8_t r, uint8_t g, uint8_t b, uint32_t ms) {
  rgbLedWrite(RGB_BUILTIN, r, g, b);
  setMotor(DIR1_PIN, PWM1_PIN, left);
  setMotor(DIR2_PIN, PWM2_PIN, right);
  Serial.printf("%-11s left=%+4d right=%+4d\n", label, left, right);
  delay(ms);
}

void setup() {
  Serial.begin(115200);

  pinMode(DIR1_PIN, OUTPUT);
  pinMode(DIR2_PIN, OUTPUT);
  digitalWrite(DIR1_PIN, LOW);
  digitalWrite(DIR2_PIN, LOW);
  ledcAttach(PWM1_PIN, PWM_FREQ, PWM_RES);
  ledcAttach(PWM2_PIN, PWM_FREQ, PWM_RES);
  ledcWrite(PWM1_PIN, 0);
  ledcWrite(PWM2_PIN, 0);

  Serial.println("\nSTEP 8: both motors");
  rgbLedWrite(RGB_BUILTIN, 32, 0, 0);
  delay(1500);
}

void loop() {
  const int16_t s = SPEED_TEST;
  drive("FORWARD",    s,  s,  0, 32,  0, 2000);
  drive("stop",       0,  0, 32,  0,  0, 1000);
  drive("BACKWARD",  -s, -s,  0,  0, 32, 2000);
  drive("stop",       0,  0, 32,  0,  0, 1000);
  drive("SPIN LEFT", -s,  s, 32, 32,  0, 2000);
  drive("stop",       0,  0, 32,  0,  0, 1000);
  drive("SPIN RIGHT", s, -s, 32,  0, 32, 2000);
  drive("stop",       0,  0, 32,  0,  0, 1000);
}
