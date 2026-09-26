// micromouse.ino - Dublin Micromouse Open 2026 firmware (ESP32-C6).
//
// Arduino IDE: board "ESP32C6 Dev Module", Tools -> USB CDC On Boot: Enabled,
// Serial Monitor 115200 baud. Library: "VL53L0X" by Pololu.
//
// Using it:
//   1. Put the mouse in the middle of the start cell, facing north (open side).
//   2. Press BOOT briefly. LED flashes white for 2 s: take your hands away.
//   3. It explores (BLUE), comes home (YELLOW), then does speed runs (GREEN)
//      and comes home after each one. RED = something went wrong.
//      With USE_DIAGONALS (config.h) it first checks any unexplored cells a
//      diagonal shortcut might need, and speed runs cut 45 degree diagonals
//      through staircases when that's faster.
//   Press BOOT at any time to stop the motors. Put it back at the start and
//   press BOOT again: it keeps the map. Hold BOOT for 2 s to forget the map.
//
// The map is saved to flash when the mouse gets home, so it survives a
// battery change. WIPE IT (hold BOOT) before running in a different maze!
//
// Type 'h' in the Serial Monitor for bench-test commands.

#include <Arduino.h>
#include <Preferences.h>
#include <VL53L0X.h>
#include <Wire.h>

#include "config.h"
#include "DiagonalPlanner.h"
#include "Hardware.h"
#include "Motion.h"
#include "Solver.h"

mm::Solver solver;
mm::DiagonalPlanner planner;  // ~90 KB, so global rather than on the stack
Preferences prefs;
bool sensorsOk = false;
bool imuOk = false;

// ============================ Map storage ===============================
void saveMaze() {
  prefs.begin("mouse", false);
  prefs.putUChar("w", solver.maze.width);
  prefs.putUChar("h", solver.maze.height);
  prefs.putBytes("walls", solver.maze.walls, sizeof(solver.maze.walls));
  prefs.putBytes("known", solver.maze.known, sizeof(solver.maze.known));
  prefs.putBytes("visited", solver.maze.visited, sizeof(solver.maze.visited));
  prefs.end();
}

bool loadMaze() {
  prefs.begin("mouse", true);
  const bool ok = prefs.getUChar("w", 0) == MAZE_WIDTH && prefs.getUChar("h", 0) == MAZE_HEIGHT &&
                  prefs.getBytesLength("walls") == sizeof(solver.maze.walls);
  if (ok) {
    prefs.getBytes("walls", solver.maze.walls, sizeof(solver.maze.walls));
    prefs.getBytes("known", solver.maze.known, sizeof(solver.maze.known));
    prefs.getBytes("visited", solver.maze.visited, sizeof(solver.maze.visited));
    // Only ever saved at the end of exploring, so go straight to speed runs.
    solver.phase = mm::Phase::READY;
    solver.exploreRounds = EXPLORE_ROUNDS;
  }
  prefs.end();
  return ok;
}

void wipeMaze() {
  prefs.begin("mouse", false);
  prefs.clear();
  prefs.end();
  solver.reset(MAZE_WIDTH, MAZE_HEIGHT);
  Serial.println("Map forgotten");
}

// ============================== Display =================================
void showIdleLed() {
  if (!sensorsOk || !imuOk) led(32, 0, 0);                         // red: hardware problem
  else if (solver.phase == mm::Phase::READY) led(0, 24, 24);       // cyan: map known
  else led(0, 0, 32);                                              // blue: nothing mapped yet
}

void showPhaseLed() {
  switch (solver.phase) {
    case mm::Phase::SPEED_RUN:        led(0, 32, 0); break;
    case mm::Phase::EXPLORE_TO_START:
    case mm::Phase::RETURN_TO_START:  led(24, 24, 0); break;
    case mm::Phase::FAILED:           led(32, 0, 0); break;
    default:                          led(0, 0, 32); break;
  }
}

// Prints the map: walls we've seen, '.' for sides not seen yet, and the
// flood-fill distance to the centre in each cell.
void printMaze() {
  const mm::Maze& m = solver.maze;
  solver.maze.flood(mm::Target::GOAL, true);
  for (int y = m.height - 1; y >= 0; y--) {
    for (int x = 0; x < m.width; x++) {
      Serial.print('+');
      Serial.print(!m.isKnown(x, y, mm::NORTH) ? " . " : m.hasWall(x, y, mm::NORTH) ? "---" : "   ");
    }
    Serial.println('+');
    for (int x = 0; x < m.width; x++) {
      Serial.print(!m.isKnown(x, y, mm::WEST) ? ':' : m.hasWall(x, y, mm::WEST) ? '|' : ' ');
      if (x == solver.x && y == solver.y) Serial.printf(" %c ", "^>v<"[solver.heading]);  // the mouse
      else if (m.distance(x, y) == mm::UNREACHABLE) Serial.print(" - ");
      else Serial.printf("%3u", m.distance(x, y));
    }
    Serial.println('|');
  }
  for (int x = 0; x < m.width; x++) Serial.print("+---");
  Serial.println('+');
  Serial.printf("Phase: %s, explore rounds: %d, at (%d,%d)\n", mm::phaseName(solver.phase),
                solver.exploreRounds, solver.x, solver.y);
}

void printSensors() {
  motion::service();
  const auto show = [](uint16_t mm) { return mm == tof::NONE ? -1 : int(mm); };
  Serial.printf("L:%4d F:%4d R:%4d mm  yaw:%7.1f deg  rate:%6.1f dps  enc L:%6ld R:%6ld\n",
                show(tof::read(tof::LEFT)), show(tof::read(tof::FRONT)), show(tof::read(tof::RIGHT)),
                imu::yawDeg, imu::rateDps, (long)encoders::left(), (long)encoders::right());
}

// ================================ Runs ==================================
// Drives wherever the solver says until it has nothing to do.
// Returns false if aborted (button / timeout / solver failed).
bool followSolver(int topPwm) {
  for (;;) {
    if (solver.wantsWalls()) {
      const motion::Walls w = motion::look();
      solver.senseWalls(w.left, w.front, w.right);
    }
    if (solver.phase == mm::Phase::EXPLORE_MARKED) mm::markDiagonalShortcuts(solver, planner);
    const mm::Phase before = solver.phase;
    const mm::Move m = solver.next();
    if (solver.phase != before) Serial.printf("Phase: %s\n", mm::phaseName(solver.phase));
    showPhaseLed();
    if (m.isStop()) return solver.phase == mm::Phase::READY;

    const int pwm = solver.phase == mm::Phase::SPEED_RUN ? topPwm : EXPLORE_PWM;
    if (!motion::execute(m, pwm)) {
      Serial.println("Stopped (button, timeout or stuck)");
      return false;
    }
  }
}

void waitForRelease() {
  while (buttonDown()) delay(10);
  delay(50);
}

// One press of BOOT: explore if needed, then speed runs.
void competitionRun() {
  if (!sensorsOk || !imuOk) {
    Serial.println("Won't run: a sensor failed at startup (see the boot messages)");
    return;
  }
  waitForRelease();
  solver.resetPose();  // it must be sitting in the start cell facing north
  for (int i = 0; i < 10; i++) {  // 2 s to get your hands clear
    led(i % 2 ? 0 : 24, i % 2 ? 0 : 24, i % 2 ? 0 : 24);
    delay(200);
  }
  imu::calibrate();
  imu::yawDeg = 0;
  motion::targetYaw = 0;

  if (solver.phase != mm::Phase::READY) {
    Serial.println("Exploring...");
    if (!followSolver(EXPLORE_PWM)) {
      motors::stop();
      waitForRelease();
      return;
    }
    saveMaze();
    Serial.printf("Explored. Best route proven: %s\n", solver.isBestRouteProven() ? "yes" : "no");
  }

  if (USE_DIAGONALS && EXPLORE_FOR_DIAGONALS && mm::markDiagonalShortcuts(solver, planner) > 0) {
    Serial.println("Exploring cells a diagonal shortcut might use...");
    solver.exploreMarked();
    if (!followSolver(EXPLORE_PWM)) {
      motors::stop();
      waitForRelease();
      return;
    }
    saveMaze();
  }

  int pwm = RUN_PWM;
  for (int run = 1; run <= SPEED_RUNS; run++) {
    motion::settle(1000);
    if (!solver.startSpeedRun()) {
      Serial.println("No fully explored route to the centre yet: exploring again");
      solver.phase = mm::Phase::EXPLORE_TO_GOAL;
      solver.maxExploreRounds = solver.exploreRounds + 1;
      if (!followSolver(EXPLORE_PWM)) break;
      continue;
    }
    const float gridTime = mm::gridPlanTime(solver, planner.costs);
    if (USE_DIAGONALS && planner.plan(solver.maze, solver.x, solver.y, solver.heading) &&
        planner.time < gridTime - 0.01f) {
      Serial.printf("Speed run %d (diagonal): %d steps, est. time %.0f vs %.0f on the grid\n",
                    run, planner.stepCount, planner.time, gridTime);
      led(0, 32, 0);
      if (!motion::executeDiagonal(planner, solver.heading, pwm)) {
        Serial.println("Diagonal run stopped (button, timeout or something too close)");
        break;
      }
      // In a goal cell now, facing along the grid: explore home as usual.
      solver.x = planner.endX;
      solver.y = planner.endY;
      solver.heading = planner.endDir;
      solver.phase = mm::Phase::RETURN_TO_START;
    } else {
      Serial.printf("Speed run %d: %d moves at PWM %d\n", run, solver.planLength, pwm);
    }
    if (!followSolver(pwm)) break;
    saveMaze();  // the trip home may have mapped more
    pwm = min(pwm + RUN_PWM_STEP, PWM_LIMIT);
  }
  motors::stop();
  waitForRelease();  // so the press that stopped it doesn't start another run
}

// ========================== Bench-test menu =============================
void printHelp() {
  Serial.println(
      "\nCommands (wheels OFF the table for p/f/l/r/a):\n"
      "  s  sensors, yaw and encoder counts    w  walls as the mouse sees them\n"
      "  m  print the map                       z  zero the encoder counts\n"
      "  g  recalibrate gyro (keep still)       p  both motors forward 1 s at EXPLORE_PWM\n"
      "  f  forward one cell    l / r / a  turn left / right / around\n"
      "  D  diagonal test on open floor: half cell, 45 right, 2 diagonal half steps,\n"
      "     45 left, half cell. Should end exactly 1 cell right and 2 cells ahead\n"
      "  G  full competition run (same as pressing BOOT)\n"
      "  x  forget the saved map");
}

void handleSerial() {
  if (!Serial.available()) return;
  const char c = Serial.read();
  switch (c) {
    case 'h': printHelp(); break;
    case 's': printSensors(); break;
    case 'w': {
      const motion::Walls w = motion::look();
      Serial.printf("walls: left=%d front=%d right=%d\n", w.left, w.front, w.right);
      break;
    }
    case 'm': printMaze(); break;
    case 'z': encoders::zero(); Serial.println("encoders zeroed"); break;
    case 'g': Serial.println("keep still..."); imu::calibrate(); Serial.println("done"); break;
    case 'p': {
      encoders::zero();
      motors::set(EXPLORE_PWM, EXPLORE_PWM);
      const uint32_t t0 = millis();
      while (millis() - t0 < 1000) motion::service();
      motors::stop();
      Serial.printf("after 1 s forward: enc L:%ld R:%ld (both should be positive and similar)\n",
                    (long)encoders::left(), (long)encoders::right());
      break;
    }
    case 'f': motion::holdCurrentHeading(); motion::forward(CELL_MM, EXPLORE_PWM); printSensors(); break;
    case 'l': motion::holdCurrentHeading(); motion::turn(90); printSensors(); break;
    case 'r': motion::holdCurrentHeading(); motion::turn(-90); printSensors(); break;
    case 'a': motion::holdCurrentHeading(); motion::turn(180); printSensors(); break;
    case 'D': {
      motion::holdCurrentHeading();
      const bool ok = motion::forward(HALF_CELL_MM, EXPLORE_PWM, false) && motion::turn(-45) &&
                      motion::forward(2 * DIAG_HALF_MM, DIAG_PWM, false) && motion::turn(45) &&
                      motion::forward(HALF_CELL_MM, EXPLORE_PWM, false);
      Serial.println(ok ? "done: measure where it stopped" : "stopped early");
      printSensors();
      break;
    }
    case 'G': competitionRun(); showIdleLed(); break;
    case 'x': wipeMaze(); showIdleLed(); break;
    default: break;
  }
}

// ============================ setup / loop ==============================
void setup() {
  Serial.begin(115200);
  const uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 2000) {}

  pinMode(PIN_BUTTON, INPUT_PULLUP);
  motors::begin();
  encoders::begin();
  led(32, 32, 32);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  sensorsOk = tof::begin();
  for (const auto& s : tof::sensors) {
    Serial.printf("ToF %s  XSHUT=GPIO%u  addr=0x%02X  %s\n", s.name, s.xshut, s.address,
                  s.ok ? "PASS" : "FAIL");
  }
  imuOk = imu::begin();
  Serial.printf("IMU  %s\n", imuOk ? "PASS" : "FAIL -> re-run Debug Kit step 2");
  Serial.println("Calibrating gyro - keep still...");
  imu::calibrate();

  solver.reset(MAZE_WIDTH, MAZE_HEIGHT);
  solver.maxExploreRounds = EXPLORE_ROUNDS;
  planner.costs.straightHalf = DIAG_TIME_HALF_CELL;
  planner.costs.diagonalHalf = DIAG_TIME_DIAG_HALF;
  planner.costs.turn45 = DIAG_TIME_TURN45;
  Serial.println(loadMaze() ? "Loaded a saved map: BOOT = speed run (hold 2 s to forget it)"
                            : "No saved map: BOOT = explore");
  printHelp();
  showIdleLed();
}

void loop() {
  motion::service();
  handleSerial();

  if (buttonDown()) {
    const uint32_t pressedAt = millis();
    while (buttonDown() && millis() - pressedAt < 2000) delay(10);
    if (buttonDown()) {  // held 2 s
      led(32, 32, 32);
      wipeMaze();
      waitForRelease();
    } else {
      competitionRun();
    }
    showIdleLed();
  }
  delay(2);
}
