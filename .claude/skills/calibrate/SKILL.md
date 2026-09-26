---
name: calibrate
description: Walk the team through calibrating the real mouse (gyro direction, motor and encoder directions, wheel size, wall distances, turns, gyro scale, wall-edge offsets), reading the real serial output at each step and proposing config.h values. Use when setting up or tuning the physical mouse.
---

# Calibrate the mouse

Follow the "Calibrating on the bench" table in `README.md`, one row at a time.
For each row:

1. Tell the human exactly how to place the mouse and whether the battery must
   be on (motor tests) and the wheels off the table.
2. Send the commands and capture the output, e.g.
   `python tools/mouse.py monitor --send s --seconds 3`
   (commands: s sensors, w walls, p motors 1 s, z zero encoders, f one cell,
   l/r/a turns, v wall-edge corrections, D diagonal test, g re-calibrate gyro).
3. Read the numbers from the saved log in `logs/` and work out the new value.
   Show the arithmetic.
4. Edit only that value in `firmware/micromouse/config.h`, with a short
   comment saying what was measured.
5. `python tools/mouse.py upload --seconds 5`, repeat the test, and confirm
   it's fixed before the next row.

After each row that works, commit it:
`git add firmware/micromouse/config.h && git commit -m "calibrate: <what> (<measured values>)"`.

Never change pins or `PWM_LIMIT`. If a reading makes no sense (e.g. a sensor
reads 0 or never changes), stop and use the hardware-debugger agent.
