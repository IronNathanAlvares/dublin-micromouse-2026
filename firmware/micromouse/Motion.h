// Motion.h - drive whole cells and turn on the spot, using the encoders for
// distance, the gyro for heading and the side sensors to stay centred.
// Every function returns false if it was aborted (BOOT pressed) or timed out.
#pragma once

#include "DiagonalPlanner.h"
#include "Hardware.h"
#include "Solver.h"

namespace motion {

// The absolute heading we're trying to hold, in degrees, anticlockwise positive.
// Turns add exactly +/-90 to it, so small gyro errors don't pile up turn after turn.
float targetYaw = 0;

// Keep the sensors fresh. Call in every control loop.
inline void service() {
  imu::update();
  tof::update();
}

// Hold the current direction from now on (e.g. before a bench test).
inline void holdCurrentHeading() {
  imu::update();
  targetYaw = imu::yawDeg;
}

// How far we are from the middle of the corridor, in mm. Positive = too far
// right, so steer left. Uses whichever side walls are there.
inline float centringError() {
  const uint16_t l = tof::read(tof::LEFT), r = tof::read(tof::RIGHT);
  const bool wallL = l < WALL_SIDE_MM, wallR = r < WALL_SIDE_MM;
  if (wallL && wallR) return (float(l) - float(r)) / 2;
  if (wallL) return float(l) - SIDE_CENTRE_MM;
  if (wallR) return SIDE_CENTRE_MM - float(r);
  return 0;
}

// Measures how far the gyro has drifted, using a side wall. While driving past
// it, a least-squares line through (distance travelled, side reading) gives the
// mouse's true average angle to the wall over that stretch. Comparing it with
// the gyro's average heading over the SAME stretch gives the gyro's error, even
// while the mouse is still straightening up after a turn. Only readings from the
// middle of cells are used (posts and wall ends confuse the sensor), readings
// that jump restart it, and a fit that isn't a clean straight line is ignored.
struct WallFit {
  float n = 0, sx = 0, sy = 0, sxx = 0, sxy = 0, syy = 0, sGyro = 0, firstX = 0, lastX = 0, lastY = 0;
  float lastAngle = 0, lastRms = 0;  // of the last fit, for printing
  uint32_t lastReadAt = 0;
  void reset() { n = sx = sy = sxx = sxy = syy = sGyro = 0; }
  // gyroRel = the gyro's heading minus the heading we're aiming for. Returns
  // true (and the gyro's error in degrees) once the wall has been followed for
  // WALL_FIT_SPAN_MM.
  bool add(float x, float y, float gyroRel, bool leftSide, float& gyroErrorDeg) {
    if (n > 0 && fabsf(y - lastY) > 12) reset();
    if (n == 0) firstX = x;
    n++; sx += x; sy += y; sxx += x * x; sxy += x * y; syy += y * y; sGyro += gyroRel;
    lastX = x; lastY = y;
    if (n < 6 || lastX - firstX < WALL_FIT_SPAN_MM) return false;
    const float vx = sxx - sx * sx / n, vxy = sxy - sx * sy / n, vy = syy - sy * sy / n;
    const float slope = vxy / vx;
    const float rms = sqrtf(fmaxf(0.0f, (vy - slope * vxy) / n));  // scatter about the line
    // Left wall getting further away = turned right (clockwise), and vice versa.
    const float wallAngle = (leftSide ? -1 : 1) * atanf(slope) * 57.29578f;
    gyroErrorDeg = constrain(sGyro / n - wallAngle, -WALL_YAW_MAX_FIX_DEG, WALL_YAW_MAX_FIX_DEG);
    lastAngle = wallAngle;
    lastRms = rms;
    reset();
    return fabsf(wallAngle) < 10 && rms < WALL_FIT_MAX_RMS_MM;
  }
};

// Pauses with the motors off while keeping the gyro integrating.
// Once the wheels have stopped moving, it also re-learns the gyro's bias:
// the mouse isn't turning, so whatever the gyro says then is drift.
inline void settle(uint32_t ms) {
  motors::stop();
  const uint32_t t0 = millis();
  int32_t lastL = encoders::left(), lastR = encoders::right();
  uint32_t stillSince = millis();
  while (millis() - t0 < ms) {
    service();
    const int32_t l = encoders::left(), r = encoders::right();
    if (l != lastL || r != lastR) stillSince = millis();
    lastL = l;
    lastR = r;
    if (millis() - stillSince > 25) imu::learnBias();
    delay(2);
  }
}

// How far past its target the last straight move really stopped (the mouse
// rolls on a little after the motors stop). The next straight move takes it
// off, so small errors don't add up cell after cell. A turn keeps the part
// that points along the new direction (all of it for 0, none for 90 degrees).
float carryMm = 0;
// Integral of the heading error while driving straight. Motors are never
// perfectly matched; this learns the steering trim that cancels the
// difference, so the mouse doesn't settle slightly off course. Kept between
// moves because the mismatch doesn't change.
float headingIntegral = 0;
bool verbose = false;  // print wall-edge corrections ('v' in the Serial Monitor)

// Drives straight for `distanceMm`, ramping speed up and down. With useWalls
// it centres between side walls, re-aligns the gyro to them, and stops early
// (at the cell centre) if a wall appears ahead. Without (diagonals, where the
// sensors see walls at 45 degrees) it steers on the gyro alone and only stops
// for something very close. gridMove = moving along the grid (not diagonally):
// then it also corrects its distance whenever a side wall starts or ends,
// because that happens at a cell boundary. startsAtEdge = starting from the
// middle of a cell edge rather than a cell centre (diagonal speed runs).
inline bool forward(float distanceMm, int topPwm, bool useWalls = true, bool gridMove = true,
                    bool startsAtEdge = false) {
  const int32_t l0 = encoders::left(), r0 = encoders::right();
  const uint32_t t0 = millis();
  const uint32_t timeoutMs = 1500 + uint32_t(distanceMm * 10);
  // Positions are measured from a cell centre along the direction of travel,
  // so cell boundaries are at +-90, 270, ... and centres at 0, 180, ...
  const float origin = startsAtEdge ? CELL_MM / 2 : 0;
  const float start = origin + carryMm;  // where we really are
  const float goal = origin + distanceMm;
  float edgeFix = 0;                           // corrections from wall edges
  bool stoppedByFrontWall = false;
  uint32_t lastUs = micros();
  uint32_t frontReadAt = 0;      // front-wall position fixes (see below)
  float lastFrontSum = 1e9f, frontFix = 0;
  WallFit fits[2];                             // left, right
  int8_t wallBeside[2] = {-1, -1};             // -1 unknown, 0 open, 1 wall
  const auto position = [&] {
    return start + edgeFix + ((encoders::left() - l0) + (encoders::right() - r0)) * 0.5f * MM_PER_TICK;
  };

  for (;;) {
    service();
    if (buttonDown() || millis() - t0 > timeoutMs) {
      motors::stop();
      return false;
    }
    const float pos = position();
    const float travelled = pos - start;
    float remaining = goal - pos;

    const uint16_t front = tof::read(tof::FRONT);
    bool frontLimited = false;
    if (useWalls && front != tof::NONE) {
      const float toWall = float(front) - FRONT_STOP_MM;
      if (toWall <= 0 && remaining > CELL_MM / 2) {
        // A wall where the map says there's none, well short of where we were
        // going: the map (or a sensor) is wrong. Stop rather than lose our place.
        motors::stop();
        return false;
      }
      if (toWall < remaining) { remaining = toWall; frontLimited = true; }
      // Walls sit on cell boundaries, so a wall ahead tells us exactly where we
      // are along the corridor (at a cell centre it reads FRONT_STOP_MM). Only
      // trusted when the reading shrinks exactly as fast as we move (a flat wall
      // square ahead, not a post or a wall seen at an angle), and capped.
      const tof::Sensor& fs = tof::sensors[tof::FRONT];
      if (gridMove && fs.readAtMs != frontReadAt) {
        frontReadAt = fs.readAtMs;
        const float sum = front + pos;  // constant while closing on a real wall
        if (front < FRONT_ALIGN_RANGE_MM && fabsf(sum - lastFrontSum) < 6) {
          const float wallAt = CELL_MM * roundf((pos + front - FRONT_STOP_MM) / CELL_MM);
          const float fix = (wallAt - front + FRONT_STOP_MM) - pos;
          if (fabsf(fix) < WALL_EDGE_MAX_FIX_MM) {
            const float step = constrain(FRONT_ALIGN_GAIN * fix, -WALL_EDGE_MAX_FIX_MM - frontFix,
                                         WALL_EDGE_MAX_FIX_MM - frontFix);
            frontFix += step;
            edgeFix += step;
          }
        }
        lastFrontSum = front < FRONT_ALIGN_RANGE_MM ? sum : 1e9f;
      }
    }
    if (!useWalls && front < DIAG_EMERGENCY_STOP_MM) {
      motors::stop();  // about to hit something: the diagonal is off course
      return false;
    }
    if (remaining <= 0) { stoppedByFrontWall = frontLimited; break; }

    if (useWalls) {
      const tof::Side sides[2] = {tof::LEFT, tof::RIGHT};
      for (int i = 0; i < 2; i++) {
        const tof::Sensor& sensor = tof::sensors[sides[i]];
        if (sensor.readAtMs == fits[i].lastReadAt) continue;  // no new reading yet
        fits[i].lastReadAt = sensor.readAtMs;
        const uint16_t d = tof::read(sides[i]);

        // A wall starting or ending beside us means we're at a cell boundary.
        const int8_t now = d < WALL_SIDE_MM ? 1 : d > WALL_SIDE_MM + WALL_EDGE_HYSTERESIS_MM ? 0 : wallBeside[i];
        if (gridMove && wallBeside[i] >= 0 && now >= 0 && now != wallBeside[i]) {
          const float boundary = CELL_MM / 2 + CELL_MM * roundf((pos - CELL_MM / 2) / CELL_MM);
          const float expected = boundary + (now == 0 ? WALL_END_OFFSET_MM : WALL_START_OFFSET_MM);
          const float fix = expected - pos;
          if (fabsf(fix) < WALL_EDGE_MAX_FIX_MM) {  // bigger = probably a post seen at an angle
            edgeFix += WALL_EDGE_GAIN * fix;
            fits[0].reset();
            fits[1].reset();
            if (verbose) Serial.printf("  wall %s on the %s: distance corrected by %+.0f mm\n",
                                       now ? "starts" : "ends", i ? "right" : "left", fix);
          }
        }
        if (now >= 0) wallBeside[i] = now;

        // Side wall beside us: measure our true angle to it and fix the gyro.
        if (d >= WALL_SIDE_MM) { fits[i].reset(); continue; }
        const float inCell = pos - CELL_MM * roundf(pos / CELL_MM);  // 0 = cell centre
        if (!gridMove || fabsf(inCell) > WALL_FIT_ZONE_MM) continue;
        float gyroError;
        const float nBefore = fits[i].n;
        const bool good = fits[i].add(travelled, float(d), imu::yawDeg - targetYaw, i == 0, gyroError);
        if (good) imu::yawDeg -= WALL_YAW_GAIN * gyroError;
        if (verbose && fits[i].n == 0 && nBefore >= 5) {
          Serial.printf("  gyro check on the %s wall: angle %+.1f, scatter %.1f mm -> %s\n", i ? "right" : "left",
                        fits[i].lastAngle, fits[i].lastRms, good ? "corrected" : "ignored");
        }
      }
    }

    const float ramp = min(travelled, remaining) * RAMP_PWM_PER_MM;
    const int pwm = min(topPwm, MIN_PWM + int(max(ramp, 0.0f)));
    const uint32_t nowUs = micros();
    const float headingError = targetYaw - imu::yawDeg;
    // Only learn the trim with no walls to centre on: next to walls the mouse
    // steers off its heading on purpose to centre itself.
    if (!useWalls) {
      headingIntegral = constrain(headingIntegral + headingError * (nowUs - lastUs) * 1e-6f,
                                  -HEADING_I_LIMIT, HEADING_I_LIMIT);
    }
    lastUs = nowUs;
    const float steer = KP_HEADING * headingError + KI_HEADING * headingIntegral +
                        (useWalls ? KP_WALL * centringError() : 0.0f);
    const int s = constrain(int(steer), -MAX_STEER, MAX_STEER);
    motors::set(pwm - s, pwm + s);  // positive steer = turn left = right wheel faster
    delay(2);
  }
  settle(60);
  if (verbose) {
    Serial.printf("  move done: %.0f of %.0f mm, %s, edge fixes %+.0f mm\n", position() - origin, goal - origin,
                  stoppedByFrontWall ? "stopped by the wall ahead" : "by distance", edgeFix);
  }
  // A front wall puts us exactly at the centre; otherwise remember the overshoot.
  carryMm = stoppedByFrontWall ? 0 : position() - goal;
  if (fabsf(carryMm) > CELL_MM / 3) carryMm = 0;
  return true;
}

// Spins on the spot by `degrees` (positive = left / anticlockwise).
inline bool turn(float degrees) {
  targetYaw += degrees;
  // Only the part of any overshoot along the new direction still matters
  // (the sideways part is fixed by wall centring).
  carryMm = fabsf(degrees) > 135 ? -carryMm : fabsf(degrees) > 60 ? 0 : carryMm * cosf(degrees * 0.0174533f);
  const uint32_t t0 = millis();
  for (;;) {
    service();
    if (buttonDown() || millis() - t0 > 3000) {
      motors::stop();
      return false;
    }
    const float error = targetYaw - imu::yawDeg;
    // Cut the motors a little early: the mouse keeps spinning for a moment.
    if (fabsf(error) < TURN_TOLERANCE_DEG + fabsf(imu::rateDps) * TURN_COAST_S) break;
    const int pwm = constrain(int(TURN_KP * fabsf(error)), TURN_MIN_PWM, TURN_MAX_PWM);
    if (error > 0) motors::set(-pwm, pwm);
    else motors::set(pwm, -pwm);
    delay(2);
  }
  settle(80);
  return true;
}

// If there's a wall straight ahead, creep forwards or backwards until the
// front sensor reads exactly what it does at the cell centre. Done before
// turning: any error along the corridor becomes a sideways error after the
// turn, and the mouse's corners need the room to spin.
inline void alignToFrontWall() {
  tof::waitFresh();
  const uint16_t first = tof::read(tof::FRONT);
  if (first >= WALL_FRONT_MM) return;  // no wall ahead (NONE is huge too)
  const uint32_t t0 = millis();
  while (millis() - t0 < 800) {
    service();
    const uint16_t f = tof::read(tof::FRONT);
    if (f >= WALL_FRONT_MM) break;
    const float err = float(f) - FRONT_STOP_MM;  // positive = too far back
    if (fabsf(err) < 3) break;
    const int pwm = err > 0 ? MIN_PWM : -MIN_PWM;
    const int s = constrain(int(KP_HEADING * (targetYaw - imu::yawDeg)), -MAX_STEER, MAX_STEER);
    motors::set(pwm - s, pwm + s);
    delay(2);
  }
  settle(40);
  carryMm = 0;
}

// Carries out one solver move: turn, then drive whole cells.
inline bool execute(const mm::Move& m, int topPwm) {
  bool ok = true;
  if (m.turn != mm::Turn::NONE) alignToFrontWall();
  switch (m.turn) {
    case mm::Turn::LEFT:   ok = turn(90); break;
    case mm::Turn::RIGHT:  ok = turn(-90); break;
    case mm::Turn::AROUND: ok = turn(180); break;
    default: break;
  }
  if (ok && m.cells > 0) ok = forward(m.cells * CELL_MM, topPwm);
  return ok;
}

// Carries out a diagonal speed-run plan that starts at a cell centre facing
// `start`. Ends at a goal cell centre facing plan.endDir.
inline bool executeDiagonal(const mm::DiagonalPlanner& plan, mm::Dir start, int topPwm) {
  int h = 2 * start;  // 8 headings, clockwise, like the planner
  int sx = 1, sy = 1;  // half-cell position (odd, odd = a cell centre); only parity matters
  for (int i = 0; i < plan.stepCount; i++) {
    const mm::DiagStep& s = plan.steps[i];
    bool ok = true;
    switch (s.kind) {
      case mm::DiagStep::TURN_LEFT_45:  ok = turn(45);  h = (h + 7) % 8; break;
      case mm::DiagStep::TURN_RIGHT_45: ok = turn(-45); h = (h + 1) % 8; break;
      case mm::DiagStep::TURN_LEFT_90:  ok = turn(90);  h = (h + 6) % 8; break;
      case mm::DiagStep::TURN_RIGHT_90: ok = turn(-90); h = (h + 2) % 8; break;
      case mm::DiagStep::HALF_STEPS: {
        const bool diagonal = h % 2 == 1;
        const bool atEdge = !(sx % 2 == 1 && sy % 2 == 1);
        ok = diagonal ? forward(s.count * DIAG_HALF_MM, min(topPwm, DIAG_PWM), false, false)
                      : forward(s.count * HALF_CELL_MM, topPwm, true, true, atEdge);
        sx += mm::DX8[h] * s.count;
        sy += mm::DY8[h] * s.count;
        break;
      }
    }
    if (!ok) return false;
  }
  return true;
}

struct Walls { bool left, front, right; };

// What the sensors say about the walls of the cell we're standing in.
inline Walls look() {
  settle(LOOK_PAUSE_MS);  // fully stopped: steadier readings, and time to re-learn gyro bias
  tof::waitFresh();
  return Walls{tof::read(tof::LEFT) < WALL_SIDE_MM,
               tof::read(tof::FRONT) < WALL_FRONT_MM,
               tof::read(tof::RIGHT) < WALL_SIDE_MM};
}

}  // namespace motion
