// DiagonalPlanner.h - the fastest speed run when 45 degree diagonals are
// allowed. Plain C++ with no Arduino code or heap use: shared by the ESP32
// firmware and the mms simulator (sim/), like Maze.h and Solver.h.
//
// Positions are in half cells ("semi-positions", the same as mms): cell (x,y)
// has its centre at (2x+1, 2y+1) and its edge middles at (2x, 2y+1),
// (2x+2, 2y+1), (2x+1, 2y) and (2x+1, 2y+2). A diagonal step goes from the
// middle of one cell edge to the middle of the next, cutting across a cell's
// corner, so a staircase of walls becomes one straight diagonal line. The
// mouse can't go diagonal from a cell centre (it would aim at a post): it
// drives half a cell to the edge, turns 45, runs the diagonal, turns 45 back.
//
// For the speed run only walls the mouse has actually seen count as open;
// with optimistic = true unseen walls count as open too, to find cells worth
// exploring (see markDiagonalShortcuts()).
#pragma once

#include <stdint.h>
#include <string.h>

#include "Solver.h"

namespace mm {

// 8 headings, clockwise from north. Heading 2*d is the grid direction d.
constexpr int8_t DX8[8] = {0, 1, 1, 1, 0, -1, -1, -1};
constexpr int8_t DY8[8] = {1, 1, 0, -1, -1, -1, 0, 1};

struct DiagStep {
  enum Kind : uint8_t { TURN_LEFT_45, TURN_RIGHT_45, TURN_LEFT_90, TURN_RIGHT_90, HALF_STEPS };
  Kind kind;
  uint8_t count;  // number of half steps, for HALF_STEPS
};

// Time of each motion. The defaults are the mms simulator's own timings
// (Window::progressRequired in the mms source); only the ratios matter.
struct DiagCosts {
  float straightHalf = 50.0f;  // half a cell along the grid
  float diagonalHalf = 70.71f; // edge middle to the next edge middle
  float turn45 = 16.66f;       // 90 degrees = two of these
  float turnPenalty = 0;       // extra per 45 degrees of turning: fewer, simpler turns
};

class DiagonalPlanner {
 public:
  static constexpr int SEMI = 2 * MAX_SIZE + 1;
  static constexpr int STATES = SEMI * SEMI * 8;
  static constexpr int MAX_STEPS = 400;
  static constexpr int MAX_POINTS = 1200;

  DiagCosts costs;

  // The last plan.
  DiagStep steps[MAX_STEPS];
  int stepCount = 0;
  uint8_t pathX[MAX_POINTS], pathY[MAX_POINTS];  // semi-positions along the route
  int pathLength = 0;
  float time = 0;
  int endX = 0, endY = 0;  // goal cell reached, facing endDir
  // What the mms simulator's score counts for this run: every turn command is
  // one turn (45 or 90), and long straight commands count less distance.
  int mmsTurns = 0;
  float mmsDistance = 0;
  Dir endDir = NORTH;

  // From the centre of cell (x,y) facing d to the centre of a goal cell,
  // facing along the grid. False if there is no route.
  bool plan(const Maze& maze, int x, int y, Dir d, bool optimistic = false) {
    const int sw = 2 * maze.width + 1, sh = 2 * maze.height + 1;
    for (int i = 0; i < STATES; i++) { cost_[i] = 1e30f; prev_[i] = -1; pos_[i] = -1; }
    heapSize_ = 0;
    const int start = id(2 * x + 1, 2 * y + 1, 2 * d);
    cost_[start] = 0;
    push(start);

    int goal = -1;
    while (heapSize_ > 0) {
      const int u = pop();
      const int h = u % 8, sy = (u / 8) % SEMI, sx = u / 8 / SEMI;
      if (sx % 2 == 1 && sy % 2 == 1 && h % 2 == 0 && maze.isGoal(sx / 2, sy / 2)) {
        goal = u;
        break;
      }
      relax(u, id(sx, sy, (h + 7) % 8), costs.turn45 + costs.turnPenalty);
      relax(u, id(sx, sy, (h + 1) % 8), costs.turn45 + costs.turnPenalty);
      const int nx = sx + DX8[h], ny = sy + DY8[h];
      if (nx >= 0 && ny >= 0 && nx < sw && ny < sh && !blocked(maze, sx, sy, h, optimistic)) {
        relax(u, id(nx, ny, h), h % 2 ? costs.diagonalHalf : costs.straightHalf);
      }
    }
    if (goal < 0) return false;
    return buildSteps(goal);
  }

  // The cell that half step i (1..pathLength-1) of the route passes through.
  void cellOfStep(int i, int& cx, int& cy) const {
    cx = (pathX[i - 1] + pathX[i]) / 4;
    cy = (pathY[i - 1] + pathY[i]) / 4;
  }

 private:
  float cost_[STATES];
  int16_t prev_[STATES];
  int16_t pos_[STATES];   // -1 never queued, -2 finished, else index in heap_
  int16_t heap_[STATES];  // binary heap of states ordered by cost_; reused when rebuilding the route
  int heapSize_ = 0;

  static int id(int sx, int sy, int h) { return (sx * SEMI + sy) * 8 + h; }

  void relax(int from, int to, float step) {
    if (pos_[to] == -2) return;
    const float c = cost_[from] + step;
    if (c >= cost_[to]) return;
    cost_[to] = c;
    prev_[to] = int16_t(from);
    push(to);
  }

  void push(int s) {  // insert, or move up after its cost dropped
    if (pos_[s] < 0) { pos_[s] = int16_t(heapSize_); heap_[heapSize_++] = int16_t(s); }
    int i = pos_[s];
    while (i > 0 && cost_[heap_[(i - 1) / 2]] > cost_[heap_[i]]) { swap(i, (i - 1) / 2); i = (i - 1) / 2; }
  }

  int pop() {
    const int top = heap_[0];
    pos_[top] = -2;
    heap_[0] = heap_[--heapSize_];
    if (heapSize_ > 0) {
      pos_[heap_[0]] = 0;
      int i = 0;
      for (;;) {
        const int l = 2 * i + 1, r = l + 1;
        int m = i;
        if (l < heapSize_ && cost_[heap_[l]] < cost_[heap_[m]]) m = l;
        if (r < heapSize_ && cost_[heap_[r]] < cost_[heap_[m]]) m = r;
        if (m == i) break;
        swap(i, m);
        i = m;
      }
    }
    return top;
  }

  void swap(int a, int b) {
    const int16_t t = heap_[a]; heap_[a] = heap_[b]; heap_[b] = t;
    pos_[heap_[a]] = int16_t(a);
    pos_[heap_[b]] = int16_t(b);
  }

  bool buildSteps(int goal) {
    int n = 0;  // states from goal back to start, stored in heap_
    for (int s = goal; s >= 0; s = prev_[s]) heap_[n++] = int16_t(s);
    stepCount = pathLength = 0;
    int pendingTurn = 0, pendingHalf = 0;  // turn in clockwise 45s
    for (int i = n - 1; i >= 0; i--) {
      const int s = heap_[i], h = s % 8;
      if (i == n - 1 || heap_[i + 1] / 8 != s / 8) {
        if (pathLength >= MAX_POINTS) return false;
        pathX[pathLength] = uint8_t(s / 8 / SEMI);
        pathY[pathLength] = uint8_t((s / 8) % SEMI);
        pathLength++;
      }
      if (i == n - 1) continue;
      if (heap_[i + 1] / 8 == s / 8) {  // turned on the spot
        if (!flushHalf(pendingHalf)) return false;
        pendingTurn += ((h - heap_[i + 1] % 8 + 8) % 8 == 1) ? 1 : -1;
      } else {
        if (!flushTurn(pendingTurn)) return false;
        pendingHalf++;
      }
    }
    if (!flushTurn(pendingTurn) || !flushHalf(pendingHalf)) return false;
    // The real time, without the turn penalty (which only shapes the route).
    time = cost_[goal];
    mmsTurns = 0;
    mmsDistance = 0;
    for (int i = 0; i < stepCount; i++) {
      if (steps[i].kind == DiagStep::HALF_STEPS) {
        const int n = steps[i].count;
        mmsDistance += n > 2 ? n / 2.0f + 1 : n;  // mms Stats::getEffectiveDistance
      } else {
        mmsTurns++;
      }
    }
    int turned45 = 0;
    for (int i = 0; i < stepCount; i++) {
      if (steps[i].kind == DiagStep::TURN_LEFT_45 || steps[i].kind == DiagStep::TURN_RIGHT_45) turned45++;
      if (steps[i].kind == DiagStep::TURN_LEFT_90 || steps[i].kind == DiagStep::TURN_RIGHT_90) turned45 += 2;
    }
    time -= turned45 * costs.turnPenalty;
    endX = (goal / 8 / SEMI) / 2;
    endY = ((goal / 8) % SEMI) / 2;
    endDir = Dir((goal % 8) / 2);
    return true;
  }

  bool add(DiagStep::Kind k, int count) {
    if (stepCount >= MAX_STEPS) return false;
    steps[stepCount++] = DiagStep{k, uint8_t(count)};
    return true;
  }
  bool flushTurn(int& turn) {
    int r = ((turn % 8) + 8) % 8;
    if (r > 4) r -= 8;
    turn = 0;
    for (; r >= 2; r -= 2) if (!add(DiagStep::TURN_RIGHT_90, 0)) return false;
    for (; r <= -2; r += 2) if (!add(DiagStep::TURN_LEFT_90, 0)) return false;
    if (r == 1) return add(DiagStep::TURN_RIGHT_45, 0);
    if (r == -1) return add(DiagStep::TURN_LEFT_45, 0);
    return true;
  }
  bool flushHalf(int& half) {
    for (; half > 255; half -= 255) if (!add(DiagStep::HALF_STEPS, 255)) return false;
    const bool ok = half == 0 || add(DiagStep::HALF_STEPS, half);
    half = 0;
    return ok;
  }

  // Mirrors mms Window::isWall(semiPos, semiDir): can the mouse move half a
  // step from (sx,sy) heading h?
  static bool blocked(const Maze& m, int sx, int sy, int h, bool optimistic) {
    const auto closed = [&](int cx, int cy, Dir d) {
      if (!m.inside(cx, cy)) return true;
      return m.hasWall(cx, cy, d) || (!optimistic && !m.isKnown(cx, cy, d));
    };
    const int cx = sx / 2, cy = sy / 2;
    if (sx % 2 == 1 && sy % 2 == 1) {  // cell centre: diagonals aim at a post
      return h % 2 == 1 || closed(cx, cy, Dir(h / 2));
    }
    if (sx % 2 == 0 && sy % 2 == 1) {  // middle of a vertical (east/west) edge
      switch (h) {
        case 0: case 4: return true;   // straight into a post
        case 2: return !m.inside(cx, cy);
        case 6: return !m.inside(cx - 1, cy);
        case 1: return closed(cx, cy, NORTH);
        case 3: return closed(cx, cy, SOUTH);
        case 7: return closed(cx - 1, cy, NORTH);
        default: return closed(cx - 1, cy, SOUTH);
      }
    }
    if (sx % 2 == 1 && sy % 2 == 0) {  // middle of a horizontal (north/south) edge
      switch (h) {
        case 2: case 6: return true;
        case 0: return !m.inside(cx, cy);
        case 4: return !m.inside(cx, cy - 1);
        case 1: return closed(cx, cy, EAST);
        case 7: return closed(cx, cy, WEST);
        case 3: return closed(cx, cy - 1, EAST);
        default: return closed(cx, cy - 1, WEST);
      }
    }
    return true;  // a corner post
  }
};

// Marks the unexplored cells on the diagonal route that would be fastest if
// they turned out to be open, for Solver::exploreMarked(). Returns how many;
// 0 means the best diagonal route through known cells can't be beaten.
inline int markDiagonalShortcuts(Solver& s, DiagonalPlanner& p) {
  memset(s.maze.marked, 0, sizeof(s.maze.marked));
  if (!p.plan(s.maze, 0, 0, NORTH, true)) return 0;
  int n = 0;
  for (int i = 1; i < p.pathLength; i++) {
    int cx, cy;
    p.cellOfStep(i, cx, cy);
    if (!s.maze.visited[cx][cy] && !s.maze.marked[cx][cy]) {
      s.maze.marked[cx][cy] = 1;
      n++;
    }
  }
  return n;
}

// Time of the solver's grid-only speed-run plan, in the same units.
inline float gridPlanTime(const Solver& s, const DiagCosts& c) {
  float t = 0;
  for (int i = 0; i < s.planLength; i++) {
    const Turn turn = s.plan[i].turn;
    t += (turn == Turn::AROUND ? 4 : turn == Turn::NONE ? 0 : 2) * c.turn45;
    t += s.plan[i].cells * 2 * c.straightHalf;
  }
  return t;
}

}  // namespace mm
