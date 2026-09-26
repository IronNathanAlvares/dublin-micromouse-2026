// =====================================================================
// STEP 1 - Is the ESP32-C6 alive?  Turns the onboard LED red.
// =====================================================================
// WIRING
//   Nothing. Just plug the ESP32-C6 into your laptop over USB.
//   (The RGB LED is built into the board on GPIO8.)
//
// WHAT YOU'LL SEE
//   LED: solid RED.
//   Serial Monitor:
//       STEP 1: LED should be RED
//       alive
//       alive
//       alive          <- a new line every second
//
// ARDUINO IDE SETTINGS
//   Board: ESP32C6 Dev Module     Tools -> USB CDC On Boot: Enabled
//   Serial Monitor: 115200 baud
//
// IF IT FAILS
//   Upload error "no upload port" -> try another USB cable (many only
//     charge), then pick the port under Tools -> Port.
//   LED red but Serial blank -> USB CDC On Boot must be Enabled.
// =====================================================================

#include <Arduino.h>

#ifndef RGB_BUILTIN
#define RGB_BUILTIN 8
#endif

void setup() {
  Serial.begin(115200);
  const uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 2000) {}

  rgbLedWrite(RGB_BUILTIN, 32, 0, 0);  // red, dimmed: 255 is blinding
  Serial.println("\nSTEP 1: LED should be RED");
}

void loop() {
  Serial.println("alive");
  delay(1000);
}
