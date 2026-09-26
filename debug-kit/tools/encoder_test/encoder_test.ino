// =====================================================================
// TOOL - Motor A encoder test
// =====================================================================
// WIRING  (step 7 wiring + the encoder)
//   Driver DIR1 -> GPIO0      Driver PWM1 -> GPIO2   (from step 7)
//   Encoder C1  -> GPIO21     Encoder C2  -> GPIO22
//   Encoder VCC -> ESP32 3V3  (NEVER the motor battery - it'll kill it)
//   Encoder GND -> ESP32 GND
//
// WHAT YOU'LL SEE
//   Serial Monitor when it works (motor starts 1 second after reset):
//       Motor A forward test
//       s = stop, g = go, z = zero encoder
//       DIR=FWD  PWM=85
//       enc=412  ticks/s=412
//       enc=829  ticks/s=417      <- ticks/s steady while it spins
//   After typing s:
//       DIR=FWD  PWM=0
//       enc=1240  ticks/s=0       <- count stops going up
//   Typing z prints "encoder zeroed" and enc starts again from 0.
//   LED: GREEN = running, RED = stopped.
//
//   Problems:
//       enc=0 ticks/s=0 while spinning -> encoder has no power, or C1 not
//                                         connected
//       ticks/s negative               -> swap C1 and C2
//       count jumps with motor stopped -> electrical noise from motor wires
// =====================================================================

#include <Arduino.h>

constexpr uint8_t DIR1_PIN = 0;
constexpr uint8_t PWM1_PIN = 2;

// Quadrature encoder channels on Motor A.
constexpr uint8_t ENC1_C1_PIN = 21;
constexpr uint8_t ENC1_C2_PIN = 22;

constexpr uint32_t PWM_FREQ = 20000;  // 20 kHz, above audible
constexpr uint8_t  PWM_RES  = 8;      // 0..255

// 170/255 of the 9V rail ~= 6V average, the N20 rating. Never exceed.
constexpr int16_t SPEED_MAX = 170;
constexpr int16_t SPEED_RUN = 85;     // half cap for a first spin

#ifndef RGB_BUILTIN
#define RGB_BUILTIN 8
#endif

int16_t g_speed = 0;

// Written from an ISR, read from loop(), so it must be volatile.
volatile int32_t g_encCount = 0;

// Fires on every rising edge of C1. C2's level at that instant tells us
// which way the shaft is turning.
void IRAM_ATTR encoderISR() {
  if (digitalRead(ENC1_C2_PIN)) {
    g_encCount++;
  } else {
    g_encCount--;
  }
}

// Sign sets direction, magnitude sets duty. Clamped both ways.
void setMotorSpeed(int16_t speed) {
  if (speed >  SPEED_MAX) speed =  SPEED_MAX;
  if (speed < -SPEED_MAX) speed = -SPEED_MAX;

  const bool forward = (speed >= 0);
  digitalWrite(DIR1_PIN, forward ? HIGH : LOW);
  ledcWrite(PWM1_PIN, forward ? speed : -speed);

  g_speed = speed;
  Serial.printf("DIR=%s  PWM=%d\n", forward ? "FWD" : "REV",
                forward ? speed : -speed);
}

void setup() {
  Serial.begin(115200);
  delay(500);  // let native USB enumerate before the first print

  // Define the direction pin before the driver can act on a floating input.
  pinMode(DIR1_PIN, OUTPUT);
  digitalWrite(DIR1_PIN, LOW);

  ledcAttach(PWM1_PIN, PWM_FREQ, PWM_RES);
  ledcWrite(PWM1_PIN, 0);   // no STBY pin on this driver, so park at zero

  // Hall encoders are open-drain and need a pull-up to give clean edges.
  pinMode(ENC1_C1_PIN, INPUT_PULLUP);
  pinMode(ENC1_C2_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC1_C1_PIN), encoderISR, RISING);

  Serial.println("\nMotor A forward test");
  Serial.println("s = stop, g = go, z = zero encoder");

  rgbLedWrite(RGB_BUILTIN, 0, 32, 0);  // green = running
  delay(1000);                          // a beat to pull your hands clear

  setMotorSpeed(SPEED_RUN);
}

void loop() {
  if (Serial.available()) {
    switch (Serial.read()) {
      case 's':
        setMotorSpeed(0);
        rgbLedWrite(RGB_BUILTIN, 32, 0, 0);  // red = stopped
        break;
      case 'g':
        setMotorSpeed(SPEED_RUN);
        rgbLedWrite(RGB_BUILTIN, 0, 32, 0);
        break;
      case 'z':
        noInterrupts();
        g_encCount = 0;
        interrupts();
        Serial.println("encoder zeroed");
        break;
    }
  }

  // Report position and rate once a second.
  static uint32_t lastReport = 0;
  static int32_t  lastCount  = 0;
  const uint32_t now = millis();
  if (now - lastReport >= 1000) {
    noInterrupts();
    const int32_t count = g_encCount;
    interrupts();

    Serial.printf("enc=%ld  ticks/s=%ld\n", count, count - lastCount);
    lastCount  = count;
    lastReport = now;
  }
}
