# Micromouse: Dublin Micromouse Open 2026

Software for our mouse (ESP32-C6, DRI0044 driver, 2× N20 encoder motors,
MPU-6050 IMU, 3× VL53L0X). The maze solver is shared: the **same** `Maze.h` /
`Solver.h` run in the mms simulator on a laptop and on the real mouse.

## Quick start: flash the mouse from the Arduino IDE

1. Get the code: `git clone https://github.com/IronNathanAlvares/dublin-micromouse-2026.git`
   (or *Code → Download ZIP* on GitHub and unzip it).
2. Arduino IDE setup (once), as in the event Debug Kit:
   - *File → Preferences → Additional Board Manager URLs*:
     `https://espressif.github.io/arduino-esp32/package_esp32_index.json`
   - *Boards Manager*: install **esp32 by Espressif Systems** (3.0.0 or newer).
   - *Library Manager*: install **VL53L0X by Pololu** (not Adafruit).
3. Open **`firmware/micromouse/micromouse.ino`**. The other files in that
   folder open as tabs automatically.
4. *Tools*: board **ESP32C6 Dev Module**, **USB CDC On Boot: Enabled**, and
   your port. Upload.
5. Serial Monitor at **115200** baud. You should see `PASS` for the three ToF
   sensors and the IMU. Type `h` for the test commands.

Plug in USB with the **battery switched off** (the ESP32 must never get USB
and the battery's 5 V at the same time). Test motors with the wheels off the
table. All pins and tuning numbers are in `config.h`. Change them there and
commit with a note of what you measured.

## What's in here

```
firmware/micromouse/   Arduino sketch that gets flashed to the ESP32-C6
  micromouse.ino         start button, LED, runs, serial bench-test menu
  config.h               ALL pins and tuning numbers
  Hardware.h             motors, encoders, ToF sensors, IMU
  Motion.h               drive N cells or half cells, diagonals, turn on the spot
  Maze.h, Solver.h       maze map + flood-fill solver (no Arduino code, shared with sim)
  DiagonalPlanner.h      fastest speed run with 45° diagonals (shared with sim)
sim/                   runs the same solver inside the mms simulator
mazes/                 the event maze (mms text file) and drawings of its fastest route
tools/fake_mms.py      headless test: runs the solver on hundreds of mazes
tools/draw_maze.py     draws a maze file with its fastest route
docs/pinout.md         the wiring the code expects
```

## How the solver works

1. **Explore to the centre** using flood fill. It assumes any wall it hasn't
   seen is open, plans the quickest route, takes one step, looks at the walls
   in the new cell and replans. It drives straight through cells it has
   already mapped.
2. **Explore back to the start** the same way. Often it takes a different
   route home, which maps more of the maze.
3. It repeats 1–2 until the best route is **proven** (no unexplored shortcut
   could beat it) or `EXPLORE_ROUNDS` is reached.
4. **Speed run**: the quickest route using only cells it has seen, with turns
   counted as costing time. Long straights become one fast move.
5. It returns home and does the next speed run.

Tested with no crashes on 300 random mazes and on 528 real competition and
practice mazes (every maze of 16×16 or smaller in `tools/mazefiles`). The speed
run was the best possible route in all of them. With the firmware's 2 explore
rounds, that held for 99 of 100 random 16×16 mazes.

---

## Part 1: The mms simulator (do this first, no hardware needed)

On this laptop it's already installed and configured:

- mms v1.2.0 is in `tools/mms/mms/mms.exe` (not in git: it's 27 MB).
- 530 real competition mazes are in `tools/mazefiles/` (not in git: get them
  with `git clone https://github.com/micromouseonline/mazefiles tools/mazefiles`).
  Seven are pre-loaded in mms: three classic
  16×16 mazes and four small practice mazes.
- The mouse **ourmouse** is registered. Pick it under **Mouse**, click
  **Build**, then **Run**. Cells visited turn blue, numbers show the
  flood-fill distance, and the speed-run route turns green. Messages appear
  in the **Run** output tab. Pick another maze from the **Maze** dropdown, or
  add more with its folder button.

Setting it up on a teammate's laptop:

1. Download `windows.zip` from https://github.com/mackorone/mms/releases
   (v1.2.0), unzip it and run `mms/mms.exe`. If SmartScreen warns you, click
   *More info → Run anyway*. They need `g++` (MinGW) on their PATH.
2. Under **Maze**, click the folder icon and open a file from `tools/mazefiles/classic`.
3. Under **Mouse**, click **+** and fill in:
   - **Name:** `ourmouse`
   - **Directory:** this repo's `sim` folder
   - **Build Command:** `g++ -std=c++17 -O2 -static -I../firmware/micromouse -o mouse.exe Main.cpp API.cpp`
   - **Run Command:** the full path to `sim\mouse.exe` in double quotes, e.g.
     `"C:/.../Micromouse Event/sim/mouse.exe"`

Quick test without the GUI (after building). This is the fastest way to check
a solver change hasn't broken anything:

```bash
python tools/fake_mms.py 100 16
python tools/fake_mms.py --files tools/mazefiles/classic tools/mazefiles/training
```

### The event maze

`mazes/dublin2026.txt` is the Dublin Micromouse Open maze, transcribed from
photos taken on the day. `mazes/dublin2026.png` is a drawing of it with the
fastest route. It's in mms as the last maze in the list.

- **Start**: the corner cell with walls on 3 sides, in the bottom left when
  you stand at the side the photos were taken from. Face the mouse out of the
  open side.
- **Goal**: the 2×2 box in the centre. Its only entrance is on its east side.
- **Double-check on the real maze** (circled in the PNG): the gap in the
  second wall from the left at row 2, and two small loose-looking pieces on
  the right at cells (13,6) and (13,4). They sat about a third of a cell off
  the grid in the photos.

**Diagonals.** Speed runs use 45° diagonals wherever they're faster
(`firmware/micromouse/DiagonalPlanner.h`, shared by the simulator and the
ESP32), e.g. straight down the staircases. Before the first speed run the
mouse also checks any unexplored cells a diagonal shortcut would need. In mms
the diagonal run takes 11987 time units against 15166 on the grid (21%
faster), the fastest possible for this maze. `mazes/dublin2026_diagonal.png`
shows the route. Run the simulator without diagonals with `mouse.exe 4 nodiag`.

On the real mouse, diagonals steer on gyro + encoders only (the side sensors
see walls at 45°), so they depend on good calibration. Before enabling them in
a real run: finish the calibration table below, then use `D` on open floor
until the mouse ends within ~1 cm of "1 cell right, 2 cells ahead". The mouse
must be under ~110 mm wide to clear the posts. If it clips posts or drifts,
set `USE_DIAGONALS = false` in `config.h` and it goes back to the grid route.

If the organisers change the maze, fix the text file and redraw it:

```bash
python tools/draw_maze.py mazes/dublin2026.txt
```

## Part 2: Arduino setup

Follow the Debug Kit setup on the event site: board **ESP32C6 Dev Module**,
**Tools → USB CDC On Boot: Enabled**, Serial Monitor at **115200**. Install
the library **VL53L0X by Pololu** (not Adafruit). Open
`firmware/micromouse/micromouse.ino` and upload.

**Before this, the hardware must pass Debug Kit steps 1–8 and the encoder
tool.** If a part fails here, go back to its Debug Kit step: that code is
known-good.

At boot the Serial Monitor should show `ToF L/F/R PASS` and `IMU PASS`. The
LED shows BLUE (no map), CYAN (map saved) or RED (a sensor failed).

## Part 3: Calibrating on the bench

Type the letters in the Serial Monitor (`h` lists them). Put every value you
change in `config.h`, re-upload, and commit it with a note of what you
measured.

| # | Test | What to change |
|---|---|---|
| 1 | Turn the mouse left by hand, type `s`. `yaw` should go **up**. | If it goes down: `GYRO_Z_SIGN = -1` |
| 2 | Wheels off the table, type `p`. Both wheels should spin **forward**. | Wrong wheel: `MOTOR_LEFT/RIGHT_REVERSED` |
| 3 | Same `p` test: both encoder counts should be **positive** and similar. | Negative: flip `ENC_LEFT/RIGHT_REVERSED`. Zero on the right: check the right encoder pins (23/11 are a guess) |
| 4 | `z`, turn one wheel exactly one revolution by hand, `s`. | `TICKS_PER_WHEEL_REV` = that count. Measure `WHEEL_DIAMETER_MM` too |
| 5 | Put the mouse in the middle of a maze cell, walls left, right and ahead. `s` a few times. | `SIDE_CENTRE_MM` = average of L and R. `FRONT_STOP_MM` = F |
| 6 | Same cell with the walls removed. `s`. | Set `WALL_SIDE_MM` / `WALL_FRONT_MM` halfway between wall and no-wall readings. Then `w` should report walls correctly |
| 7 | On the floor: `f`. It should drive one cell (180 mm) straight. | Drifts: `KP_HEADING`/`KP_WALL`. Wrong distance: step 4. Stalls: raise `MIN_PWM` |
| 8 | `l`, `r`, `a`: 90°, 90°, 180°. | Overshoots/wobbles: lower `TURN_KP` or `TURN_MAX_PWM`. Stalls short: raise `TURN_MIN_PWM` |

The mouse turns on the spot, so the wheel axle should sit in the middle of
the cell when it's centred.

## Part 4: Running it

1. On a mini practice maze, first set `MAZE_WIDTH`/`MAZE_HEIGHT` to its size.
2. Place the mouse **in the middle of the start cell, facing out of the open
   side** (that's "north" to the solver).
3. Press **BOOT** briefly. The LED flashes white for 2 s, so get your hands clear.
4. BLUE = exploring, YELLOW = heading home, GREEN = speed run.
5. **Press BOOT to stop** at any time. Put it back at the start and press BOOT
   again: it keeps the map.
6. **Hold BOOT for 2 s to forget the map.** Do this every time you move to a
   different maze! The map is saved to flash so it survives battery swaps.

Use `m` in the Serial Monitor to print the map the mouse has built. It's the
fastest way to spot a sensor that reports walls wrongly.

## Team workflow

Use the steps in the event's *C2 Git and GitHub Basics* guide. Commit after
every calibration step that works, e.g. `calibrate: wall thresholds from maze A`.
Keep one owner per file at a time: typically one person on `config.h` +
`Motion.h` (tuning), one on `Solver.h` (tested in the simulator first).

## Ideas if there's time left

- **Diagonals**: the event has a diagonal mini maze. mms supports
  `turnLeft45`/`moveForwardHalf`, so try diagonal speed runs in the simulator first.
- **Smoother speed runs**: curved turns instead of stopping to spin.
- **Back-wall alignment**: at dead ends, reverse gently into the wall to reset position and heading.
- **Tune `Costs`** in `Maze.h` to match how long your real turns take compared with a cell.
