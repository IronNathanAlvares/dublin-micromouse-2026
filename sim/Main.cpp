// Main.cpp - runs the mouse's maze solver inside the mms simulator.
//
// The brains live in ../firmware/micromouse/Maze.h and Solver.h, the exact
// files that get flashed to the ESP32. This file only swaps the real sensors
// and motors for mms commands.
//
// Speed runs use 45 degree diagonals (DiagonalPlanner.h, also shared with the
// ESP32) whenever that's faster.
//
// mms settings (Mouse -> "+"):
//   Directory:      <this sim folder>
//   Build Command:  g++ -std=c++17 -O2 -static -I../firmware/micromouse -o mouse.exe Main.cpp API.cpp
//   Run Command:    mouse.exe
//   Optional args:  mouse.exe <explore rounds> nodiag

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>

#include "API.h"
#include "DiagonalPlanner.h"
#include "Solver.h"

using namespace mm;

namespace {

constexpr int SPEED_RUNS = 2;
const char DIR_CHAR[4] = {'n', 'e', 's', 'w'};

void log(const std::string& text) { std::cerr << text << std::endl; }

// Shows each cell's flood-fill distance, only re-sending cells that changed.
class DistanceDisplay {
 public:
  void update(const Maze& maze) {
    for (int x = 0; x < maze.width; x++) {
      for (int y = 0; y < maze.height; y++) {
        const uint16_t d = maze.distance(x, y);
        const uint32_t key = uint32_t(d) + 1;  // +1 so 0 can mean "nothing shown yet"
        if (shown_[x][y] == key) continue;
        shown_[x][y] = key;
        API::setText(x, y, d == UNREACHABLE ? "-" : std::to_string(d));
      }
    }
  }

 private:
  uint32_t shown_[MAX_SIZE][MAX_SIZE] = {};
};

// Draws what the mouse has learned about its current cell.
void drawWalls(const Solver& s) {
  for (int d = 0; d < 4; d++) {
    if (s.maze.isKnown(s.x, s.y, Dir(d)) && s.maze.hasWall(s.x, s.y, Dir(d))) {
      API::setWall(s.x, s.y, DIR_CHAR[d]);
    }
  }
  if (!s.maze.isGoal(s.x, s.y)) API::setColor(s.x, s.y, 'B');
}

void drawPlan(const Solver& s) {
  int x = s.x, y = s.y;
  Dir h = s.heading;
  API::setColor(x, y, 'G');
  for (int i = 0; i < s.planLength; i++) {
    h = applyTurn(h, s.plan[i].turn);
    for (int c = 0; c < s.plan[i].cells; c++) {
      x += DX[h];
      y += DY[h];
      API::setColor(x, y, 'G');
    }
  }
}

// Big (~90 KB), so one shared instance rather than one on the stack per use.
DiagonalPlanner planner;

void drawDiagonalPlan(const DiagonalPlanner& d) {
  for (int i = 1; i < d.pathLength; i++) {
    int cx, cy;
    d.cellOfStep(i, cx, cy);
    API::setColor(cx, cy, 'G');
  }
}

bool driveDiagonal(const DiagonalPlanner& d) {
  for (int i = 0; i < d.stepCount; i++) {
    const DiagStep& s = d.steps[i];
    switch (s.kind) {
      case DiagStep::TURN_LEFT_45:  API::turnLeft45(); break;
      case DiagStep::TURN_RIGHT_45: API::turnRight45(); break;
      case DiagStep::TURN_LEFT_90:  API::turnLeft(); break;
      case DiagStep::TURN_RIGHT_90: API::turnRight(); break;
      case DiagStep::HALF_STEPS:
        if (!API::moveForwardHalf(s.count)) return false;
        break;
    }
  }
  return true;
}

bool drive(const Move& m) {
  switch (m.turn) {
    case Turn::LEFT:   API::turnLeft(); break;
    case Turn::RIGHT:  API::turnRight(); break;
    case Turn::AROUND: API::turnRight(); API::turnRight(); break;
    default: break;
  }
  return m.cells == 0 || API::moveForward(m.cells);
}

}  // namespace

int main(int argc, char** argv) {
  Solver solver;
  solver.maxExploreRounds = argc > 1 ? std::atoi(argv[1]) : 4;  // tools/fake_mms.py tries others
  const bool useDiagonals = !(argc > 2 && std::string(argv[2]) == "nodiag");
  planner.costs.turnPenalty = argc > 3 ? float(std::atof(argv[3])) : planner.costs.turnPenalty;
  solver.reset(API::mazeWidth(), API::mazeHeight());
  log("Maze " + std::to_string(solver.maze.width) + "x" + std::to_string(solver.maze.height));

  DistanceDisplay display;
  Phase lastPhase = solver.phase;
  int speedRuns = 0;
  int cellsDriven = 0;

  for (;;) {
    if (API::wasReset()) {
      API::ackReset();
      solver.resetPose();
      log("Reset pressed: back at the start, keeping the map");
    }

    if (solver.wantsWalls()) {
      solver.senseWalls(API::wallLeft(), API::wallFront(), API::wallRight());
      drawWalls(solver);
    }

    if (useDiagonals && solver.phase == Phase::EXPLORE_MARKED) markDiagonalShortcuts(solver, planner);
    const Move m = solver.next();
    display.update(solver.maze);

    if (solver.phase != lastPhase) {
      log(std::string("Phase: ") + phaseName(solver.phase) + "  (cells driven so far: " +
          std::to_string(cellsDriven) + ")");
      lastPhase = solver.phase;
    }

    if (m.isStop()) {
      if (solver.phase != Phase::READY || speedRuns >= SPEED_RUNS) break;
      if (useDiagonals && speedRuns == 0 && markDiagonalShortcuts(solver, planner) > 0) {
        log("Checking unexplored cells a diagonal shortcut might use");
        solver.exploreMarked();
        continue;
      }
      log(std::string("Best route proven optimal: ") + (solver.isBestRouteProven() ? "yes" : "no"));
      if (!solver.startSpeedRun()) {
        log("No fully explored route to the centre!");
        break;
      }
      speedRuns++;
      API::clearAllColor();

      DiagonalPlanner& diag = planner;
      const double gridTime = gridPlanTime(solver, diag.costs);
      if (useDiagonals && diag.plan(solver.maze, solver.x, solver.y, solver.heading) &&
          diag.time < gridTime - 0.01) {
        drawDiagonalPlan(diag);
        log("Speed run " + std::to_string(speedRuns) + " (diagonal): " +
            std::to_string(diag.stepCount) + " commands, mms time " +
            std::to_string(int(diag.time + 0.5)) + ", mms run score " +
            std::to_string(int(diag.mmsTurns + diag.mmsDistance + 0.5)) + " (" +
            std::to_string(diag.mmsTurns) + " turns) (grid-only route: " +
            std::to_string(int(gridTime + 0.5)) + ")");
        if (!driveDiagonal(diag)) {
          log("CRASHED on the diagonal speed run - the planner is wrong");
          return 1;
        }
        // Now in a centre cell, facing along the grid: head home as usual.
        solver.x = diag.endX;
        solver.y = diag.endY;
        solver.heading = diag.endDir;
        solver.phase = Phase::RETURN_TO_START;
      } else {
        drawPlan(solver);
        int turns = 0;
        float distance = 0;
        for (int i = 0; i < solver.planLength; i++) {
          turns += solver.plan[i].turn == Turn::AROUND ? 2 : solver.plan[i].turn == Turn::NONE ? 0 : 1;
          const int half = 2 * solver.plan[i].cells;  // mms counts half steps
          distance += half > 2 ? half / 2.0f + 1 : half;
        }
        log("Speed run " + std::to_string(speedRuns) + ": " + std::to_string(solver.planLength) +
            " moves, cost " + std::to_string(solver.planCost) + ", mms time " +
            std::to_string(int(gridTime + 0.5)) + ", mms run score " +
            std::to_string(int(turns + distance + 0.5)) + " (" + std::to_string(turns) + " turns)");
      }
      lastPhase = solver.phase;
      continue;
    }

    if (!drive(m)) {
      log("CRASHED into a wall - the map or the solver is wrong");
      return 1;
    }
    cellsDriven += m.cells;
  }

  log(solver.phase == Phase::FAILED ? "Gave up: no route to the target" : "Done");
  return 0;
}
