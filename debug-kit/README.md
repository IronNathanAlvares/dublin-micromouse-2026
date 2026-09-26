# Micromouse Debug Kit (ESP32-C6)

Small, single-purpose Arduino sketches for finding out *which part* of your
micromouse is broken. Each one tests one thing and tells you clearly whether
it passed.

**Rule of thumb:** when something doesn't work, stop testing the whole robot.
Flash the smallest sketch that tests the broken part, confirm it, then add
the next piece.

## One-time setup (Arduino IDE)

1. **Board package:** File → Preferences → Additional Board Manager URLs:
   `https://espressif.github.io/arduino-esp32/package_esp32_index.json`
   Then Boards Manager → install **esp32 by Espressif Systems**, version
   **3.0.0 or newer** (older versions don't support the C6).
2. **Board:** Tools → Board → **ESP32C6 Dev Module**
3. **Tools → USB CDC On Boot → Enabled** (otherwise Serial Monitor stays blank)
4. **Serial Monitor:** **115200 baud**, **No Line Ending**
5. Library (only for the ToF steps): **VL53L0X by Pololu**. Careful:
   `Adafruit_VL53L0X` is a *different* library and won't work with this code.
   The IMU needs no library.

## How to use it

Work through the steps **in order**, connecting one part at a time. Each step
keeps all the wiring from the step before and adds one thing. The **exact
wiring each sketch expects is written at the top of the file.**

To run a step: open its `.ino` (or paste it into a fresh sketch), upload, then
press **RESET** with Serial Monitor open so you see the startup PASS/FAIL.

| Step | Sketch | You connect | Pass looks like |
|---|---|---|---|
| 1 | `01_led_red` | Nothing, just USB | Onboard LED goes red |
| 2 | `02_imu` | MPU-6050 | `PASS`, heading follows the board as you turn it |
| 3 | `03_tof_1` | Left ToF | `PASS`, distance changes when you wave a hand |
| 4 | `04_tof_2` | + Front ToF | Both `PASS` |
| 5 | `05_tof_3` | + Right ToF | All three `PASS` |
| 6 | `06_tof_3_imu` | Nothing new | All 4 devices `PASS`, one combined line |
| 7 | `07_motor_1` | Driver + left motor | Motor follows the LED: green fwd, blue back |
| 8 | `08_motors_2` | + right motor | Forward, back, spin left, spin right |

**If a step fails, the problem is almost always the thing you just added.**
Unplug it, re-run the previous step to confirm everything else still works,
then check the new part's wiring against the top of its file.

The motor steps don't need Serial. The LED colour tells you what the motors
should be doing, so you can check them even when Serial isn't set up right.

### IMU visualizer (`imu_visualizer/`)

A web page that shows your IMU working: a 3D board that turns and tilts with
the real one, a compass, live values and graphs.

1. Wire the IMU like step 2 and upload `imu_visualizer/imu_visualizer.ino`.
2. **Close the Arduino Serial Monitor** (only one program can use the USB port).
3. Open `imu_visualizer/index.html` in **Chrome or Edge** (double-click it).
4. Click **Connect board** and pick the ESP32's COM port.
5. Keep the board still for one second, then turn and tilt it.

No board handy? Click **Try demo** to see it with simulated motion. If the 3D
board turns or tilts the wrong way, tick **Flip turn / pitch / roll**. Firefox
and Safari can't connect to USB boards.

### Extra tools (`tools/`)

| Sketch | Use it when |
|---|---|
| `i2c_scan` | Any I2C step fails. Lists every device found + checks SDA/SCL idle HIGH |
| `pin_test` | Motor dead and you want to prove GPIO0/GPIO2 toggle. **Unplug the motor first** |
| `motor_sweep` | Motor hums but won't turn. Ramps power up so you see where it starts |
| `encoder_test` | Checking motor encoder wiring (C1→GPIO21, C2→GPIO22). Prints counts |

## Reference wiring

Pin numbers are defined at the top of every sketch. If your robot is wired
differently, change them there.

| Signal | ESP32-C6 |
|---|---|
| Motor A DIR / PWM | GPIO0 / GPIO2 |
| Motor B DIR / PWM | GPIO3 / GPIO10 |
| Motor A encoder C1 / C2 | GPIO21 / GPIO22 |
| I2C SDA / SCL | GPIO6 / GPIO7 |
| ToF XSHUT left / front / right | GPIO18 / GPIO19 / GPIO20 |
| Onboard RGB LED | GPIO8 (don't use it for anything else) |

**Avoid GPIO 4, 5, 8, 9, 15.** They're strapping pins on the C6 and can make
the board boot strangely.

Power:
- Motor driver **VM** → battery +
- Battery −, driver **GND**, ESP32 **GND** → all joined together
- Sensor and encoder **VCC** → ESP32 **3V3**. **Never the motor battery.**

Motor PWM is capped at **170/255**, about 6V average from a 9V battery, which
is what N20 motors are rated for. Don't raise the cap.

## Troubleshooting (problems people actually hit)

### "Failed uploading: no upload port provided"
The board isn't selected or detected.
- Try a different USB cable. Many cables only carry power, not data.
- Tools → Port → pick the COM port.
- Still nothing? Hold **BOOT**, tap **RESET**, release **BOOT**, then upload.

### Serial Monitor is blank, or only shows `ESP-ROM:esp32c6-...`
- Baud must be **115200**.
- **Tools → USB CDC On Boot → Enabled**, then upload again.
- Press **RESET** after opening the monitor.
- Check it actually uploaded: the Output tab should say `Hash of data verified`.

### I uploaded new code but the board does the old thing
Opening or downloading a new `.ino` doesn't change the tab you're editing.
Select all in the editor, paste the new code, save, upload. The LED sketches
make this easy to spot because each one shows different colours.

### Motor does nothing
Run `07_motor_1`. If the LED cycles through its colours, **the code is fine** and the
problem is wiring:
1. **Is PWM connected?** (This was the cause the first time we debugged it.)
2. Is DIR connected?
3. Driver GND joined to ESP32 GND?
4. Battery connected to VM, and not flat?
5. Does the driver have a logic **VCC** pin? It needs 3.3V.
6. Does the driver have an **STBY / EN / SLP** pin? It must be HIGH.

Still stuck? `tools/motor_sweep` ramps the power up. If the motor hums but never
turns, the battery is sagging or the duty is too low.

### All I2C sensors fail / `i2c_scan` finds 0 devices
Look at the `idle SDA= SCL=` line:
- **0 / 0**: the lines aren't reaching the sensors, or a sensor has no power.
  Check VCC/GND on every board. Remove all sensors except one and add them
  back one at a time.
- **1 / 1** but nothing found: wires are connected but something's off. Run
  `02_imu`, which tries SDA/SCL both ways round and both IMU addresses.

Common causes:
- **SDA and SCL swapped.** The idle check can't catch this. `02_imu` can.
- MPU-6050 wired to **XDA/XCL** instead of **SDA/SCL**.
- MPU **INT** pin wired somewhere it shouldn't be. Leave INT unconnected.
- Breadboard power rail split halfway along.
- Broken jumper wire. Swap it.
