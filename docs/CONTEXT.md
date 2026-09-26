# Hardware context pack

The event's *C3 Working with AI Agents* guide says to keep the facts an agent
(or a teammate) can't see in the repo. This is that file. Paste it, or point
the agent at it, before asking for help with the mouse.

## The hardware

| Item | Exact part | Notes |
|---|---|---|
| Controller | ESP32-C6-DevKitC-1 | 3.3 V logic. Never 5 V into a GPIO |
| Motor driver | DFRobot DRI0044 (TB6612FNG) | DIR + PWM per motor. VCC = 3V3, VM = battery |
| Motors | 2x GA12-N20, 6 V, ~500 rpm, Hall encoders | Encoder VCC = 3V3. PWM capped at 170/255 (6 V from 9 V) |
| IMU | DFRobot SEN0142 (MPU-6050), I2C 0x68 | Only gyro Z (yaw) is used. Read by raw registers, no library |
| Distance | 3x VL53L0X (GY-530), I2C 0x30 L / 0x31 F / 0x29 R | Addresses assigned at boot via XSHUT |
| Power | 2S Li-ion + 5 V buck | Never USB and the 5 V rail at the same time |

Pins: see [pinout.md](pinout.md). All of them are in `firmware/micromouse/config.h`.

## Software versions (known good)

| | Version |
|---|---|
| Arduino IDE / arduino-cli | IDE 2.x, arduino-cli 1.5.1 |
| Board package | esp32 by Espressif Systems 3.3.12 |
| Board | ESP32C6 Dev Module, USB CDC On Boot: Enabled (`esp32:esp32:esp32c6:CDCOnBoot=cdc`) |
| Distance sensor library | VL53L0X by Pololu 1.3.1 (not Adafruit) |
| Simulator | mms v1.2.0 |

## Build, flash, monitor

```bash
python tools/mouse.py build
python tools/mouse.py upload
python tools/mouse.py monitor --send s w
python tools/mouse.py debug 06_tof_3_imu
python tools/mouse.py test
```

`build` compiles only, and `upload` flashes and opens the monitor. `monitor` saves everything to `logs/` (`--send s w` types commands for you). `debug` flashes a Debug Kit step. `test` runs all offline tests, no board needed.

Serial: 115200 baud. Type `h` for the bench commands.

## Known-good results

| What | Result | Where |
|---|---|---|
| Firmware compiles for ESP32-C6 | 26% flash, 40% RAM, no warnings | `python tools/mouse.py build` |
| All Debug Kit sketches compile | 13/13 (encoder_test has 2 harmless warnings) | `debug-kit/` |
| Solver, mms protocol | 100/100 random + 528 real mazes, best route every time | `tools/fake_mms.py` |
| Real firmware on the virtual robot, grid runs | 24/24 full competitions, 0 wrong walls | `sim/robot/` |
| Real firmware on the virtual robot, diagonal runs | clips posts, so `USE_DIAGONALS = false` | `sim/robot/` |
| On the real mouse | **not tested yet** | add results here |

## How to ask for help (from the C3 guide)

1. Say what you expected and what happened, and paste the **real** serial log
   from `logs/`.
2. Ask for **one small change**, and say what must not change (pins, other files).
3. Test on the mouse, commit if it works (`git commit -m "fix: ..."`), revert if not.
4. Stop and have a human look if anything gets hot, the board resets when the
   motors start, or the minimal Debug Kit sketch also fails. That's wiring,
   not code.

Example:

```
Read docs/CONTEXT.md and firmware/micromouse/config.h first.
Problem: after 'l' the mouse turns about 97 degrees instead of 90. Log below.
Only change config.h calibration values. Tell me which one and why.
<paste logs/2026-09-26_15-02-11.txt>
```
