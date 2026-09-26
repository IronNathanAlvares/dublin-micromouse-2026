// Maze.h - what the mouse knows about the maze, plus the path finding.
//
// Plain C++ with no Arduino code, so the SAME file runs in the mms simulator
// (sim/) and on the ESP32 (this sketch). Try ideas in the simulator first,
// then flash them.
//
// Coordinates follow mms: (0,0) is the start cell in the bottom-left corner,
// x grows to the east, y grows to the north, and the mouse starts facing north.
#pragma once

#include <stdint.h>
#include <string.h>

namespace mm {

constexpr int MAX_SIZE = 16;              // classic 16 x 16 maze
constexpr uint16_t UNREACHABLE = 0xFFFF;

enum Dir : uint8_t { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };
constexpr int8_t DX[4] = {0, 1, 0, -1};
constexpr int8_t DY[4] = {1, 0, -1, 0};
inline Dir leftOf(Dir d)  { return Dir((d + 3) & 3); }
inline Dir rightOf(Dir d) { return Dir((d + 1) & 3); }
inline Dir behind(Dir d)  { return Dir((d + 2) & 3); }

enum class Turn : uint8_t { NONE, LEFT, RIGHT, AROUND };

// The rotation that takes heading `from` to heading `to`.
inline Turn turnBetween(Dir from, Dir to) {
  switch ((int(to) - int(from) + 4) & 3) {
    case 0:  return Turn::NONE;
    case 1:  return Turn::RIGHT;
    case 2:  return Turn::AROUND;
    default: return Turn::LEFT;
  }
}

inline Dir applyTurn(Dir d, Turn t) {
  switch (t) {
    case Turn::LEFT:   return leftOf(d);
    case Turn::RIGHT:  return rightOf(d);
    case Turn::AROUND: return behind(d);
    default:           return d;
  }
}

// One instruction for the robot: rotate in place, then drive `cells` forward.
struct Move {
  Turn turn = Turn::NONE;
  uint8_t cells = 0;
  bool isStop() const { return turn == Turn::NONE && cells == 0; }
};

// Relative time of each manoeuvre, used to plan the speed run. The units don't
// matter, only the ratios: with these numbers a 90 degree turn "costs" the same
// as driving 1.5 cells, so the planner prefers long straights over zig-zags.
struct Costs {
  uint16_t cell = 2;
  uint16_t turn90 = 3;
  uint16_t turn180 = 5;
};

enum class Target : uint8_t { GOAL, START, MARKED };

class Maze {
 public:
  int width = MAX_SIZE;
  int height = MAX_SIZE;

  // Bit d of walls[x][y] = there is a wall on side d of cell (x,y).
  // Bit d of known[x][y] = side d has actually been seen by a sensor.
  // Public so the firmware can save them to flash.
  uint8_t walls[MAX_SIZE][MAX_SIZE];
  uint8_t known[MAX_SIZE][MAX_SIZE];
  uint8_t visited[MAX_SIZE][MAX_SIZE];
  uint8_t marked[MAX_SIZE][MAX_SIZE];  // cells we want to go and look at (Target::MARKED)

  void init(int w, int h) {
    width = (w > 0 && w <= MAX_SIZE) ? w : MAX_SIZE;
    height = (h > 0 && h <= MAX_SIZE) ? h : MAX_SIZE;
    forgetInteriorWalls();
    memset(visited, 0, sizeof(visited));
    memset(marked, 0, sizeof(marked));
  }

  // Wipes everything learned but keeps the outer boundary. Used when the map
  // contradicts itself (a sensor must have lied).
  void forgetInteriorWalls() {
    memset(walls, 0, sizeof(walls));
    memset(known, 0, sizeof(known));
    for (int x = 0; x < width; x++) {
      setWall(x, 0, SOUTH, true);
      setWall(x, height - 1, NORTH, true);
    }
    for (int y = 0; y < height; y++) {
      setWall(0, y, WEST, true);
      setWall(width - 1, y, EAST, true);
    }
  }

  bool inside(int x, int y) const { return x >= 0 && y >= 0 && x < width && y < height; }
  bool hasWall(int x, int y, Dir d) const { return (walls[x][y] >> d) & 1; }
  bool isKnown(int x, int y, Dir d) const { return (known[x][y] >> d) & 1; }

  // Records a wall (or the lack of one) on both sides of the boundary.
  void setWall(int x, int y, Dir d, bool present) {
    setSide(x, y, d, present);
    const int nx = x + DX[d], ny = y + DY[d];
    if (inside(nx, ny)) setSide(nx, ny, behind(d), present);
  }

  // Optimistic: unseen walls count as open (for exploring).
  // Pessimistic: only walls we've seen to be open count (for the speed run).
  bool canMove(int x, int y, Dir d, bool optimistic) const {
    if (hasWall(x, y, d)) return false;
    if (!optimistic && !isKnown(x, y, d)) return false;
    return inside(x + DX[d], y + DY[d]);
  }

  // The goal is the centre: 2x2 cells for an even size, 1 cell for odd.
  bool isGoal(int x, int y) const {
    return inCentre(x, width) && inCentre(y, height);
  }
  bool isTarget(Target t, int x, int y) const {
    if (t == Target::MARKED) return marked[x][y];
    return t == Target::GOAL ? isGoal(x, y) : (x == 0 && y == 0);
  }

  // Flood fill: dist(x,y) = number of cells from (x,y) to the target.
  void flood(Target t, bool optimistic) {
    uint8_t queue[MAX_SIZE * MAX_SIZE];
    int head = 0, tail = 0;
    for (int x = 0; x < width; x++) {
      for (int y = 0; y < height; y++) {
        dist_[x][y] = UNREACHABLE;
        if (isTarget(t, x, y)) {
          dist_[x][y] = 0;
          queue[tail++] = uint8_t(x * MAX_SIZE + y);
        }
      }
    }
    while (head < tail) {
      const int x = queue[head] / MAX_SIZE, y = queue[head] % MAX_SIZE;
      head++;
      for (int d = 0; d < 4; d++) {
        if (!canMove(x, y, Dir(d), optimistic)) continue;
        const int nx = x + DX[d], ny = y + DY[d];
        if (dist_[nx][ny] != UNREACHABLE) continue;
        dist_[nx][ny] = dist_[x][y] + 1;
        queue[tail++] = uint8_t(nx * MAX_SIZE + ny);
      }
    }
  }

  uint16_t distance(int x, int y) const { return dist_[x][y]; }

  // Fastest route from (x,y) facing `heading` to the target, counting turns as
  // well as distance (Dijkstra over cell+heading). Writes up to maxMoves moves
  // into `out` (may be null) and returns the route's total number of moves,
  // or -1 if there is no route.
  int planPath(int x, int y, Dir heading, Target t, bool optimistic,
               const Costs& costs, Move* out, int maxMoves, uint32_t* totalCost) const {
    // static: these ~22 KB are far too much for the ESP32 loop() stack.
    static uint16_t cost[STATES];
    static int16_t prev[STATES];
    static int16_t path[STATES];
    static uint32_t heap[HEAP_CAP];  // (cost << 16) | state, smallest on top
    int heapSize = 0;

    for (int i = 0; i < STATES; i++) { cost[i] = INF; prev[i] = -1; }
    const int start = stateOf(x, y, heading);
    cost[start] = 0;
    heapPush(heap, heapSize, start, 0);

    int goal = -1;
    while (heapSize > 0) {
      const uint32_t top = heapPop(heap, heapSize);
      const int u = int(top & 0xFFFF);
      if ((top >> 16) != cost[u]) continue;  // stale entry, already found a cheaper way
      const int ux = u / 4 / MAX_SIZE, uy = u / 4 % MAX_SIZE;
      const Dir ud = Dir(u % 4);
      if (isTarget(t, ux, uy)) { goal = u; break; }

      if (canMove(ux, uy, ud, optimistic)) {
        relax(heap, heapSize, cost, prev, u, stateOf(ux + DX[ud], uy + DY[ud], ud), costs.cell);
      }
      relax(heap, heapSize, cost, prev, u, stateOf(ux, uy, leftOf(ud)), costs.turn90);
      relax(heap, heapSize, cost, prev, u, stateOf(ux, uy, rightOf(ud)), costs.turn90);
      relax(heap, heapSize, cost, prev, u, stateOf(ux, uy, behind(ud)), costs.turn180);
    }
    if (goal < 0) return -1;
    if (totalCost) *totalCost = cost[goal];

    int len = 0;
    for (int s = goal; s >= 0; s = prev[s]) path[len++] = int16_t(s);

    // Walk the states from start to goal and squash them into moves.
    int n = 0;
    uint8_t runCells = 0;
    Dir facing = heading;
    for (int i = len - 2; i >= 0; i--) {
      const int a = path[i + 1], b = path[i];
      if (a / 4 == b / 4) continue;  // turned on the spot: folded into the next move
      const Dir d = Dir(b % 4);
      if (n > 0 && d == facing && runCells < 255) {
        runCells++;
        if (out && n <= maxMoves) out[n - 1].cells = runCells;
      } else {
        if (out && n < maxMoves) out[n] = Move{turnBetween(facing, d), 1};
        n++;
        runCells = 1;
        facing = d;
      }
    }
    return n;
  }

 private:
  static constexpr int STATES = MAX_SIZE * MAX_SIZE * 4;  // every cell x every heading
  static constexpr int HEAP_CAP = STATES * 4 + 1;         // one push per improved edge at most
  static constexpr uint16_t INF = 0xFFFF;

  uint16_t dist_[MAX_SIZE][MAX_SIZE];

  static bool inCentre(int v, int size) {
    return size % 2 ? v == size / 2 : (v == size / 2 - 1 || v == size / 2);
  }
  static int stateOf(int x, int y, Dir d) { return (x * MAX_SIZE + y) * 4 + d; }

  static void relax(uint32_t* heap, int& heapSize, uint16_t* cost, int16_t* prev,
                    int from, int to, uint16_t step) {
    const uint32_t c = uint32_t(cost[from]) + step;
    if (c >= cost[to]) return;
    cost[to] = uint16_t(c);
    prev[to] = int16_t(from);
    heapPush(heap, heapSize, to, c);
  }

  static void heapPush(uint32_t* heap, int& size, int state, uint32_t c) {
    if (size >= HEAP_CAP) return;
    int i = size++;
    heap[i] = (c << 16) | uint32_t(state);
    while (i > 0 && heap[(i - 1) / 2] > heap[i]) {
      const uint32_t tmp = heap[i]; heap[i] = heap[(i - 1) / 2]; heap[(i - 1) / 2] = tmp;
      i = (i - 1) / 2;
    }
  }

  static uint32_t heapPop(uint32_t* heap, int& size) {
    const uint32_t top = heap[0];
    heap[0] = heap[--size];
    int i = 0;
    for (;;) {
      const int l = 2 * i + 1, r = l + 1;
      int m = i;
      if (l < size && heap[l] < heap[m]) m = l;
      if (r < size && heap[r] < heap[m]) m = r;
      if (m == i) break;
      const uint32_t tmp = heap[i]; heap[i] = heap[m]; heap[m] = tmp;
      i = m;
    }
    return top;
  }

  void setSide(int x, int y, Dir d, bool present) {
    known[x][y] |= uint8_t(1 << d);
    if (present) walls[x][y] |= uint8_t(1 << d);
    else walls[x][y] &= uint8_t(~(1 << d));
  }
};

}  // namespace mm
