# Notes for AI coding agents

Micromouse for the Dublin Micromouse Open 2026 (ESP32-C6, Arduino). Read
`docs/CONTEXT.md` (hardware, versions, commands) and `docs/pinout.md` first.

- `firmware/micromouse/` is the Arduino sketch. `Maze.h`, `Solver.h` and
  `DiagonalPlanner.h` are plain C++ shared with the simulators: no Arduino
  calls in them.
- All pins and tuning values live in `config.h`. Never hard-code a pin
  elsewhere, and don't change pins without the team's wiring agreeing.
- Before committing, run `python tools/mouse.py test`. It runs the solver
  tests and the full virtual-robot competition with the real firmware, and it
  must print ALL TESTS PASSED. For firmware changes also run
  `python tools/mouse.py build`.
- The virtual robot (`sim/robot/robot_sim.cpp`) fakes the hardware.
  `robot_sim.exe <maze> <seed> -v` prints true vs believed position, which is
  the fastest way to debug motion code. Test several seeds and mazes.
- Keep changes small, one behaviour at a time (event guide C3). Ask for real
  serial logs (`logs/`) before guessing at hardware faults.
- Don't touch `debug-kit/`: it's the event's official code, kept as-is.

## Ready-made helpers (Claude Code)

- `/flash`: build, upload, check that every sensor says PASS at boot.
- `/calibrate`: go through the README calibration table with real serial readings.
- `/debug-kit`: run the event's Debug Kit steps in order to find a broken part.
- `/test`: every offline check before a commit.
- Agent `hardware-debugger`: diagnose a real-mouse fault from serial logs.
- Agent `sim-tester`: check a change on the virtual robot across mazes and seeds.
