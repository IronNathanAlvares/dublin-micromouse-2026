// robot_sim.cpp - runs the REAL firmware (firmware/micromouse/micromouse.ino,
// unmodified) on a laptop against a virtual robot in a virtual maze.
//
// Fakes only the hardware, at the level the firmware talks to it:
//   motors      DIR/PWM pins -> wheel speeds with lag, deadband and one weaker wheel
//   encoders    real quadrature A/B pin changes into the firmware's interrupt code
//   MPU-6050    register-level I2C device with gyro bias and noise
//   VL53L0X x3  ranges by ray-casting the maze's real walls and posts, with noise
//   BOOT button pressed once to start a competition run
// and fails loudly if the robot's body ever touches a wall or post.
//
//   g++ -std=c++17 -O2 -Isim/robot/fake -Ifirmware/micromouse -o robot_sim.exe sim/robot/robot_sim.cpp
//   robot_sim.exe mazes/dublin2026.txt [seed] [-v]
#include "micromouse.ino"  // the firmware, compiled against the fake headers

#include <fstream>
#include <random>
#include <string>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace world {

// ---- The virtual robot (a plausible, calibrated mouse) ----
constexpr double TRACK_MM = 78;          // wheel to wheel
constexpr double BODY_LENGTH = 90;       // collision box, centred on the axle
constexpr double BODY_WIDTH = 80;
constexpr double TRUE_WHEEL_MM = 34.3;   // 1% bigger than config.h thinks
constexpr double DEADBAND = 25;          // PWM below this doesn't move
constexpr double MM_S_PER_PWM = 4.8;     // ~700 mm/s at the 170 cap
constexpr double RIGHT_STRENGTH = 0.97;  // right motor 3% weaker
constexpr double MOTOR_TAU_S = 0.03;     // speed lag (the driver brakes at PWM 0)
constexpr double GYRO_BIAS_DPS = 0.8;
constexpr double GYRO_NOISE_DPS = 0.3;
constexpr double TOF_NOISE_MM = 2.0;
// Sensor positions (x forward, y left) and directions, matching config.h's
// SIDE_CENTRE_MM (55) and FRONT_STOP_MM (50) with 84 mm from centre to a wall face.
constexpr double SENSOR[3][3] = {{0, 29, 90}, {34, 0, 0}, {0, -29, -90}};  // L, F, R

constexpr double CELL = 180, HALF_WALL = 6;

struct Box { double x0, y0, x1, y1; };
std::vector<Box> boxes;
int mazeW = 0, mazeH = 0;
std::vector<std::vector<int>> truth;  // truth[x][y] = wall bits (1 N, 2 E, 4 S, 8 W)

double x = 90, y = 90, th = M_PI / 2;  // mm, radians (anticlockwise, 0 = east)
double vl = 0, vr = 0, distL = 0, distR = 0;
uint32_t dutyL = 0, dutyR = 0;
int pins[64] = {};
void (*isr[64])() = {};
long quarterL = 0, quarterR = 0;
uint64_t tUs = 0;
uint64_t buttonDownFrom = 0, buttonDownTo = 0;
int tofNext = 0;
bool tofClaimed[3] = {};
uint8_t mpuReg = 0;
double gyroDrift = 0;
std::mt19937 rng(1);
bool verbose = false;
bool atLineStart = true;

// Goal bookkeeping
bool inGoal = false;
std::vector<double> goalArrivals;
double leftStartAt = -1;
std::vector<double> runTimes;

bool isGoal(int cx, int cy) {
  auto centre = [](int v, int n) { return n % 2 ? v == n / 2 : (v == n / 2 - 1 || v == n / 2); };
  return centre(cx, mazeW) && centre(cy, mazeH);
}

bool loadMaze(const std::string& path) {
  std::ifstream in(path);
  std::vector<std::string> lines;
  for (std::string l; std::getline(in, l);) {
    if (!l.empty() && l.back() == '\r') l.pop_back();
    if (l.find_first_not_of(' ') != std::string::npos) lines.push_back(l);
  }
  if (lines.size() < 3) return false;
  mazeH = int(lines.size() - 1) / 2;
  mazeW = int(lines[0].size() - 1) / 4;
  truth.assign(mazeW, std::vector<int>(mazeH, 0));
  for (int yy = 0; yy < mazeH; yy++) {
    auto row = [&](int i) { std::string s = lines[(mazeH - 1 - yy) * 2 + i]; s.resize(4 * mazeW + 1, ' '); return s; };
    const std::string top = row(0), mid = row(1), bottom = row(2);
    for (int xx = 0; xx < mazeW; xx++) {
      if (top[4 * xx + 1] == '-') truth[xx][yy] |= 1;
      if (mid[4 * xx + 4] == '|') truth[xx][yy] |= 2;
      if (bottom[4 * xx + 1] == '-') truth[xx][yy] |= 4;
      if (mid[4 * xx] == '|') truth[xx][yy] |= 8;
    }
  }
  for (int xx = 0; xx < mazeW; xx++) {
    for (int yy = 0; yy < mazeH; yy++) {
      const double X0 = xx * CELL, Y0 = yy * CELL, X1 = X0 + CELL, Y1 = Y0 + CELL;
      if (truth[xx][yy] & 1) boxes.push_back({X0, Y1 - HALF_WALL, X1, Y1 + HALF_WALL});
      if (truth[xx][yy] & 2) boxes.push_back({X1 - HALF_WALL, Y0, X1 + HALF_WALL, Y1});
      if (truth[xx][yy] & 4) boxes.push_back({X0, Y0 - HALF_WALL, X1, Y0 + HALF_WALL});
      if (truth[xx][yy] & 8) boxes.push_back({X0 - HALF_WALL, Y0, X0 + HALF_WALL, Y1});
    }
  }
  for (int i = 0; i <= mazeW; i++) {
    for (int j = 0; j <= mazeH; j++) {
      boxes.push_back({i * CELL - HALF_WALL, j * CELL - HALF_WALL, i * CELL + HALF_WALL, j * CELL + HALF_WALL});
    }
  }
  return true;
}

double raycast(double ox, double oy, double dx, double dy) {
  double best = 1e9;
  for (const Box& b : boxes) {
    double t0 = 0, t1 = 1e9;
    const double o[2] = {ox, oy}, d[2] = {dx, dy}, lo[2] = {b.x0, b.y0}, hi[2] = {b.x1, b.y1};
    bool hit = true;
    for (int a = 0; a < 2 && hit; a++) {
      if (std::fabs(d[a]) < 1e-12) {
        if (o[a] < lo[a] || o[a] > hi[a]) hit = false;
      } else {
        double ta = (lo[a] - o[a]) / d[a], tb = (hi[a] - o[a]) / d[a];
        if (ta > tb) std::swap(ta, tb);
        t0 = std::max(t0, ta);
        t1 = std::min(t1, tb);
        if (t0 > t1) hit = false;
      }
    }
    if (hit && t0 < best) best = t0;
  }
  return best;
}

void crash(const char* what) {
  std::printf("\n*** CRASH at t=%.2fs: %s. Robot at (%.0f, %.0f) mm, heading %.1f deg, cell (%d,%d)\n",
              tUs / 1e6, what, x, y, th * 180 / M_PI, int(x / CELL), int(y / CELL));
  std::exit(2);
}

void checkCollision() {
  const double c = std::cos(th), s = std::sin(th);
  for (int i = 0; i <= 8; i++) {
    for (int side = 0; side < 4; side++) {
      const double f = i / 8.0 - 0.5;
      double px, py;  // point on the body outline, robot frame
      switch (side) {
        case 0: px = BODY_LENGTH / 2; py = f * BODY_WIDTH; break;
        case 1: px = -BODY_LENGTH / 2; py = f * BODY_WIDTH; break;
        case 2: px = f * BODY_LENGTH; py = BODY_WIDTH / 2; break;
        default: px = f * BODY_LENGTH; py = -BODY_WIDTH / 2; break;
      }
      const double wx = x + px * c - py * s, wy = y + px * s + py * c;
      for (const Box& b : boxes) {
        if (wx > b.x0 && wx < b.x1 && wy > b.y0 && wy < b.y1) crash("body hit a wall or post");
      }
    }
  }
}

double wheelTarget(uint32_t duty, uint8_t dirPin) {
  const double sign = pins[dirPin] ? 1 : -1;
  return sign * std::max(0.0, double(duty) - DEADBAND) * MM_S_PER_PWM;
}

// Quadrature: forward steps go (A,B) = 01 11 10 00, so on each A edge A==B,
// which the firmware counts as +1 (the Debug Kit's convention).
void stepEncoder(long& q, long target, uint8_t pinA, uint8_t pinB) {
  static const int A[4] = {0, 1, 1, 0}, B[4] = {1, 1, 0, 0};
  while (q != target) {
    q += q < target ? 1 : -1;
    const int k = int(((q % 4) + 4) % 4);
    const bool aChanged = pins[pinA] != A[k];
    pins[pinB] = B[k];
    pins[pinA] = A[k];
    if (aChanged && isr[pinA]) isr[pinA]();
  }
}

void physics(double dt) {
  vl += (wheelTarget(dutyL, PIN_DIR_LEFT) - vl) * std::min(1.0, dt / MOTOR_TAU_S);
  vr += (wheelTarget(dutyR, PIN_DIR_RIGHT) * RIGHT_STRENGTH - vr) * std::min(1.0, dt / MOTOR_TAU_S);
  const double v = (vl + vr) / 2, w = (vr - vl) / TRACK_MM;
  x += v * std::cos(th) * dt;
  y += v * std::sin(th) * dt;
  th += w * dt;
  distL += vl * dt;
  distR += vr * dt;
  const double quarterMm = M_PI * TRUE_WHEEL_MM / TICKS_PER_WHEEL_REV / 2;
  stepEncoder(quarterL, long(std::floor(distL / quarterMm)), PIN_ENC_LEFT_A, PIN_ENC_LEFT_B);
  // The right motor is mirrored, so its encoder counts backwards (hence
  // ENC_RIGHT_REVERSED = true in config.h).
  stepEncoder(quarterR, long(std::floor(-distR / quarterMm)), PIN_ENC_RIGHT_A, PIN_ENC_RIGHT_B);
  std::normal_distribution<double> walk(0, 0.0003);  // ~0.013 dps per sqrt(s): a pessimistic MPU-6050
  gyroDrift += walk(rng);
}

void track() {
  const int cx = int(x / CELL), cy = int(y / CELL);
  static int lastCx = -1, lastCy = -1;
  if (verbose && (cx != lastCx || cy != lastCy)) {
    // True heading vs what the firmware believes (its yaw 0 = the start heading, north).
    std::printf("[%7.2fs]   sim: cell (%d,%d) offset (%+.0f,%+.0f) mm  true yaw %+.1f  firmware yaw %+.1f  target %+.1f  L/F/R %d/%d/%d\n",
                tUs / 1e6, cx, cy, std::fmod(x, CELL) - 90, std::fmod(y, CELL) - 90,
                th * 180 / M_PI - 90, imu::yawDeg, motion::targetYaw,
                int(tof::sensors[0].mm), int(tof::sensors[1].mm), int(tof::sensors[2].mm));
    lastCx = cx;
    lastCy = cy;
  }
  const bool nowGoal = isGoal(cx, cy);
  if (nowGoal && !inGoal) {
    goalArrivals.push_back(tUs / 1e6);
    if (leftStartAt >= 0) runTimes.push_back(tUs / 1e6 - leftStartAt);
    leftStartAt = -1;
  }
  inGoal = nowGoal;
  static bool moving = false;
  const double speed = std::fabs(vl) + std::fabs(vr);
  if (moving && speed < 40 && verbose) {
    std::printf("[%7.2fs]   sim: STOPPED in cell (%d,%d), offset (%+.0f,%+.0f) mm from its centre, true yaw %+.1f\n",
                tUs / 1e6, cx, cy, std::fmod(x, CELL) - 90, std::fmod(y, CELL) - 90, th * 180 / M_PI - 90);
  }
  moving = speed > 120 ? true : (speed < 40 ? false : moving);
  static bool inStart = true;
  const bool nowStart = cx == 0 && cy == 0;
  if (inStart && !nowStart) leftStartAt = tUs / 1e6;
  inStart = nowStart;
}

}  // namespace world

// ---------------------------- hardware hooks ----------------------------
namespace simhw {
using namespace world;

uint64_t nowUs() { return tUs; }

void advanceUs(uint64_t us) {
  static uint64_t lastCollision = 0;
  static bool busy = false;  // the sim's own bookkeeping must never re-enter
  if (busy) return;
  busy = true;
  const uint64_t end = tUs + us;
  while (tUs < end) {
    const uint64_t step = std::min<uint64_t>(500, end - tUs);
    physics(step / 1e6);
    tUs += step;
    if (tUs - lastCollision >= 2000) { checkCollision(); track(); lastCollision = tUs; }
  }
  busy = false;
}

void pinWrite(uint8_t pin, int level) { pins[pin] = level ? 1 : 0; }
int pinRead(uint8_t pin) {
  if (pin == PIN_BUTTON) return (tUs >= buttonDownFrom && tUs < buttonDownTo) ? 0 : 1;
  return pins[pin];
}
void pwmWrite(uint8_t pin, uint32_t duty) {
  if (pin == PIN_PWM_LEFT) dutyL = duty;
  if (pin == PIN_PWM_RIGHT) dutyR = duty;
}
void led(uint8_t, uint8_t, uint8_t) {}
void attachIsr(uint8_t pin, void (*fn)()) { isr[pin] = fn; }

int i2cWrite(uint8_t address, const uint8_t* data, int n) {
  advanceUs(150);
  if (address != 0x68) return 2;
  if (n >= 1) mpuReg = data[0];
  return 0;
}
int i2cRead(uint8_t address, uint8_t* out, int n) {
  advanceUs(100);
  if (address != 0x68) return 0;
  for (int i = 0; i < n; i++) out[i] = 0;
  if (mpuReg == 0x47 && n >= 2) {  // GYRO_ZOUT at +/-1000 dps
    std::normal_distribution<double> noise(0, GYRO_NOISE_DPS);
    const double dps = (vr - vl) / TRACK_MM * 180 / M_PI + GYRO_BIAS_DPS + gyroDrift + noise(rng);
    const long raw = std::lround(std::max(-1000.0, std::min(1000.0, dps)) * 32.8);
    out[0] = uint8_t((raw >> 8) & 0xFF);
    out[1] = uint8_t(raw & 0xFF);
  }
  return n;
}

int tofClaim() {
  const uint8_t xshut[3] = {PIN_XSHUT_LEFT, PIN_XSHUT_FRONT, PIN_XSHUT_RIGHT};
  for (int i = 0; i < 3; i++) {
    if (pins[xshut[i]] && !tofClaimed[i]) { tofClaimed[i] = true; return i; }
  }
  return -1;
}

uint16_t tofRange(int sensor) {
  const double c = std::cos(th), s = std::sin(th);
  const double ox = x + SENSOR[sensor][0] * c - SENSOR[sensor][1] * s;
  const double oy = y + SENSOR[sensor][0] * s + SENSOR[sensor][1] * c;
  double best = 1e9;
  for (double spread : {-8.0, 0.0, 8.0}) {  // VL53L0X sees a cone, not a line
    const double a = th + (SENSOR[sensor][2] + spread) * M_PI / 180;
    best = std::min(best, raycast(ox, oy, std::cos(a), std::sin(a)));
  }
  std::normal_distribution<double> noise(0, TOF_NOISE_MM);
  best += noise(rng);
  return best > 1200 ? 8190 : uint16_t(std::max(0.0, best));
}

int serialAvailable() { return 0; }
int serialRead() { return -1; }
void serialWrite(const char* text) {
  if (!verbose) return;
  for (const char* p = text; *p; p++) {
    if (atLineStart) std::printf("[%7.2fs] ", tUs / 1e6);
    std::putchar(*p);
    atLineStart = *p == '\n';
  }
}
}  // namespace simhw

// ------------------------------- driver ---------------------------------
int main(int argc, char** argv) {
  using namespace world;
  if (argc < 2 || !loadMaze(argv[1])) {
    std::printf("usage: robot_sim <maze.txt> [seed] [-v]\n");
    return 1;
  }
  for (int i = 2; i < argc; i++) {
    if (std::string(argv[i]) == "--turns") continue;
    if (std::string(argv[i]) == "-v") { verbose = true; motion::verbose = true; }
    else rng.seed(unsigned(std::atoi(argv[i])));
  }
  // Placed by hand: roughly, not perfectly, in the middle of the start cell.
  std::uniform_real_distribution<double> jitter(-1, 1);
  x += 5 * jitter(rng);
  y += 5 * jitter(rng);
  th += 2 * M_PI / 180 * jitter(rng);

  setup();
  if (!sensorsOk || !imuOk) { std::printf("FAIL: firmware reported a sensor failure at boot\n"); return 1; }

  if (argc > 2 && std::string(argv[argc - 1]) == "--turns") {  // bench test of turn() alone
    imu::calibrate();
    motion::holdCurrentHeading();
    const double trueStart = th * 180 / M_PI, fwStart = imu::yawDeg;
    for (float deg : {90.0f, 90.0f, -90.0f, -90.0f, 180.0f, -180.0f, 45.0f, -45.0f}) {
      const uint64_t t0 = tUs;
      const bool ok = motion::turn(deg);
      motion::settle(300);  // let everything stop, gyro still integrating
      std::printf("turn %+5.0f: %s in %.2fs  true %+7.1f  firmware %+7.1f  target %+7.1f\n", deg,
                  ok ? "ok     " : "TIMEOUT", (tUs - t0) / 1e6, th * 180 / M_PI - trueStart,
                  imu::yawDeg - fwStart, motion::targetYaw - fwStart);
    }
    return 0;
  }
  buttonDownFrom = tUs + 500000;  // press BOOT for 0.2 s
  buttonDownTo = buttonDownFrom + 200000;
  while (tUs < buttonDownTo + 1000000) loop();
  // loop() only returns from the competition run when it's finished.

  int wrong = 0, known = 0;
  for (int cx = 0; cx < mazeW; cx++) {
    for (int cy = 0; cy < mazeH; cy++) {
      for (int d = 0; d < 4; d++) {
        if (!solver.maze.isKnown(cx, cy, mm::Dir(d))) continue;
        known++;
        if (solver.maze.hasWall(cx, cy, mm::Dir(d)) != bool(truth[cx][cy] & (1 << d))) wrong++;
      }
    }
  }
  std::printf("Finished at t=%.1fs. Phase: %s. Map: %d wall sides known, %d wrong.\n", tUs / 1e6,
              mm::phaseName(solver.phase), known, wrong);
  std::printf("Reached the centre %zu times; start-to-centre runs:", goalArrivals.size());
  for (double t : runTimes) std::printf(" %.1fs", t);
  std::printf("\nEnded at cell (%d,%d), %.0f mm / %.0f mm from its centre\n", int(x / CELL), int(y / CELL),
              std::fabs(std::fmod(x, CELL) - 90), std::fabs(std::fmod(y, CELL) - 90));
  const bool ok = wrong == 0 && runTimes.size() >= size_t(1 + SPEED_RUNS) && solver.phase == mm::Phase::READY;
  std::printf("%s\n", ok ? "PASS" : "FAIL");
  return ok ? 0 : 1;
}
