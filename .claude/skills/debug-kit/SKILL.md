---
name: debug-kit
description: Run the event's Debug Kit checks (01_led_red to 08_motors_2, plus i2c_scan, encoder_test, motor_sweep, pin_test) on the connected board one at a time, telling the human what to wire and judging PASS/FAIL from the real output. Use to find which part of the mouse is broken.
---

# Debug Kit walkthrough

The sketches are in `debug-kit/`, unchanged from the event. The wiring each one
expects is in the comment at the top of its `.ino`, and the pass criteria are
in `debug-kit/README.md`. Read both before each step.

For each step, in order (01 to 08), stopping at the first failure:
1. Tell the human what must be connected and what must NOT be (extra ToF
   boards with no XSHUT control clash at 0x29). Motor steps: battery on,
   wheels off the table.
2. `python tools/mouse.py debug <sketch> --seconds 8`, e.g. `02_imu`.
3. Compare the saved log with the "WHAT YOU'LL SEE" section of that sketch.
   Motor steps print little: ask the human to confirm the LED colours and
   wheel directions.
4. On a failure, use the matching tool (`i2c_scan` for any I2C problem,
   `encoder_test`, `motor_sweep`, and `pin_test` only with the motor
   unplugged) and the "If something fails" table in `debug-kit/README.md`.
   Say the most likely wire to check.

Finish with a short table: step, result, evidence.
