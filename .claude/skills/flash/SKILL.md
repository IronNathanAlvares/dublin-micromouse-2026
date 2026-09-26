---
name: flash
description: Build and upload the micromouse firmware to the connected ESP32-C6, then check the boot messages (all sensors PASS). Use when asked to flash, upload or program the mouse.
---

# Flash the mouse

1. Safety check with the human first: battery switched OFF (USB and the 5 V
   rail must never power the ESP32 together), wheels off the table, Arduino
   IDE Serial Monitor closed.
2. `python tools/mouse.py ports`: there must be a board. If not: data-capable
   USB cable, or hold BOOT, tap RESET, release BOOT.
3. `python tools/mouse.py upload --seconds 6`: builds, uploads and saves the
   boot output to `logs/`.
4. Read the log. Expected:
   `ToF L ... PASS`, `ToF F ... PASS`, `ToF R ... PASS`, `IMU  PASS`, then
   "No saved map" or "Loaded a saved map". Any FAIL: say which Debug Kit
   step to run (`/debug-kit`) and stop.
5. Report what the boot said and the LED colour the human should see
   (blue = no map, cyan = map saved, red = sensor failure).
