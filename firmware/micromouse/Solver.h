// Solver.h - decides where the mouse goes next. No Arduino code: shared with
// the mms simulator exactly like Maze.h.
//
// How a competition goes with this solver:
//   1. EXPLORE_TO_GOAL   flood-fill towards the centre, one cell at a time,
//                        sensing walls in every cell.
//   2. EXPLORE_TO_START  flood-fill back home. Takes a different route if it
//                        looks shorter, so it maps more of the maze.
//   (repeat 1-2 until the best route is proven or maxExploreRounds is hit)
//   3. READY             at the start. Call startSpeedRun().
//   4. SPEED_RUN         follow the fastest route through KNOWN cells only,
//                        several cells per move.
//   5. RETURN_TO_START   explore back home, then READY again.
//
// The caller's loop is always:
//   if (solver.wantsWalls()) solver.senseWalls(left, front, right);
//   Move m = solver.next();          // also updates the solver's position
//   if (m.isStop()) ...; else drive m (turn first, then m.cells forward; may be 0)
#pragma once

#include "Maze.h"

namespace mm {

enum class Phase : uint8_t {
  EXPLORE_TO_GOAL,
  EXPLORE_TO_START,
  EXPLORE_MARKED,  // visit the cells in maze.marked, then go home
  READY,
  SPEED_RUN,
  RETURN_TO_START,
  FAILED,
};

inline const char* phaseName(Phase p) {
  switch (p) {
    case Phase::EXPLORE_TO_GOAL:  return "explore to centre";
    case Phase::EXPLORE_TO_START: return "explore back to start";
    case Phase::EXPLORE_MARKED:   return "explore marked cells";
    case Phase::READY:            return "ready for speed run";
    case Phase::SPEED_RUN:        return "speed run";
    case Phase::RETURN_TO_START:  return "return to start";
    default:                      return "FAILED";
  }
}

class Solver {
 public:
  static constexpr int MAX_MOVES = MAX_SIZE * MAX_SIZE;

  Maze maze;
  Costs costs;
  int maxExploreRounds = 2;  // centre-and-back trips before settling for the best known route

  int x = 0, y = 0;
  Dir heading = NORTH;
  Phase phase = Phase::EXPLORE_TO_GOAL;
  int exploreRounds = 0;

  // The current speed-run plan (also handy for drawing it).
  Move plan[MAX_MOVES];
  int planLength = 0;
  int planIndex = 0;
  uint32_t planCost = 0;

  // Start from scratch in a w x h maze.
  void reset(int w, int h) {
    maze.init(w, h);
    exploreRounds = 0;
    phase = Phase::EXPLORE_TO_GOAL;
    resetPose();
  }

  // The mouse was put back in the start cell facing north by hand.
  // Keeps the map; abandons whatever run was in progress.
  void resetPose() {
    x = 0;
    y = 0;
    heading = NORTH;
    planLength = planIndex = 0;
    if (phase == Phase::SPEED_RUN || phase == Phase::RETURN_TO_START) phase = Phase::READY;
    if (phase == Phase::EXPLORE_TO_START || phase == Phase::FAILED) phase = Phase::EXPLORE_TO_GOAL;
    if (phase == Phase::EXPLORE_MARKED) phase = Phase::READY;
  }

  bool wantsWalls() const {
    return phase == Phase::EXPLORE_TO_GOAL || phase == Phase::EXPLORE_TO_START ||
           phase == Phase::EXPLORE_MARKED || phase == Phase::RETURN_TO_START;
  }

  // Wall readings in the current cell, relative to the way the mouse faces.
  void senseWalls(bool left, bool front, bool right) {
    maze.setWall(x, y, leftOf(heading), left);
    maze.setWall(x, y, heading, front);
    maze.setWall(x, y, rightOf(heading), right);
    maze.visited[x][y] = 1;
  }

  // The next move. Assumes the robot will carry it out and updates x/y/heading.
  // Returns a stop move (isStop()) when there is nothing to do.
  Move next() {
    for (;;) {
      switch (phase) {
        case Phase::EXPLORE_TO_GOAL:
          if (maze.isGoal(x, y)) { phase = Phase::EXPLORE_TO_START; continue; }
          return exploreStep(Target::GOAL);

        case Phase::EXPLORE_TO_START:
          if (atStart()) {
            if (heading != NORTH) return turnToFace(NORTH);  // ready for the next run
            exploreRounds++;
            phase = (exploreRounds >= maxExploreRounds || isBestRouteProven())
                        ? Phase::READY : Phase::EXPLORE_TO_GOAL;
            continue;
          }
          return exploreStep(Target::START);

        case Phase::EXPLORE_MARKED: {
          maze.marked[x][y] = 0;
          bool any = false;
          for (int i = 0; i < maze.width; i++)
            for (int j = 0; j < maze.height; j++) any = any || maze.marked[i][j];
          if (any) {
            Move first;
            if (maze.planPath(x, y, heading, Target::MARKED, true, costs, &first, 1, nullptr) >= 0) {
              return exploreStep(Target::MARKED);
            }
            memset(maze.marked, 0, sizeof(maze.marked));  // walled off: forget them
          }
          phase = Phase::RETURN_TO_START;
          continue;
        }

        case Phase::SPEED_RUN:
          if (planIndex < planLength) {
            const Move m = plan[planIndex++];
            apply(m);
            return m;
          }
          phase = Phase::RETURN_TO_START;
          continue;

        case Phase::RETURN_TO_START:
          if (atStart()) {
            if (heading != NORTH) return turnToFace(NORTH);
            phase = Phase::READY;
            continue;
          }
          return exploreStep(Target::START);

        default:  // READY, FAILED
          return Move{};
      }
    }
  }

  // Plans the fastest route to the centre using only walls we've actually seen.
  // Returns false if no fully-known route exists yet.
  bool startSpeedRun() {
    planLength = maze.planPath(x, y, heading, Target::GOAL, false, costs, plan, MAX_MOVES, &planCost);
    planIndex = 0;
    if (planLength <= 0) { planLength = 0; return false; }
    phase = Phase::SPEED_RUN;
    return true;
  }

  // True when the best route through known cells is as good as the best route
  // assuming every unseen wall is open, i.e. more exploring can't beat it.
  bool isBestRouteProven() const {
    uint32_t known = 0, optimistic = 0;
    if (maze.planPath(0, 0, NORTH, Target::GOAL, false, costs, nullptr, 0, &known) < 0) return false;
    maze.planPath(0, 0, NORTH, Target::GOAL, true, costs, nullptr, 0, &optimistic);
    return known == optimistic;
  }

  // A turn that leaves the mouse facing `d` (e.g. north again after coming home).
  Move turnToFace(Dir d) {
    const Move m{turnBetween(heading, d), 0};
    heading = d;
    return m;
  }

  // Go and look at the cells in maze.marked (set them first), then return home.
  void exploreMarked() { phase = Phase::EXPLORE_MARKED; }

  bool atStart() const { return x == 0 && y == 0; }

 private:
  void apply(const Move& m) {
    heading = applyTurn(heading, m.turn);
    x += DX[heading] * m.cells;
    y += DY[heading] * m.cells;
  }

  // Flood-fill exploring: plan the fastest route to the target pretending every
  // unseen wall is open, then take its first step. Walls found on the way
  // change the plan. Using the same turn-aware costs as the speed run means we
  // explore exactly the routes the speed run would like to use.
  Move exploreStep(Target t) {
    maze.flood(t, true);  // only for displaying distances
    Move first;
    if (maze.planPath(x, y, heading, t, true, costs, &first, 1, nullptr) < 0) {
      // The map says we're walled in, so some reading was wrong. Start mapping again.
      maze.forgetInteriorWalls();
      maze.flood(t, true);
      if (maze.planPath(x, y, heading, t, true, costs, &first, 1, nullptr) < 0) {
        phase = Phase::FAILED;
        return Move{};
      }
    }
    // Stop in every new cell to look at its walls, but drive straight through
    // cells we've already mapped.
    const Dir d = applyTurn(heading, first.turn);
    uint8_t cells = 1;
    while (cells < first.cells && maze.visited[x + DX[d] * cells][y + DY[d] * cells]) cells++;
    const Move m{first.turn, cells};
    apply(m);
    return m;
  }
};

}  // namespace mm
