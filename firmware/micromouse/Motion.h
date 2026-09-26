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

// Pauses with the motors off while keeping the gyro integrating.
inline void settle(uint32_t ms) {
  motors::stop();
  const uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    service();
    delay(2);
  }
}

// Drives straight for `distanceMm`, ramping speed up and down. With useWalls
// it centres between side walls and stops early (at the cell centre) if a wall
// appears ahead. Without (diagonals, where the sensors see walls at 45
// degrees) it steers on the gyro alone and only stops for something very close.
inline bool forward(float distanceMm, int topPwm, bool useWalls = true) {
  const int32_t l0 = encoders::left(), r0 = encoders::right();
  const uint32_t t0 = millis();
  const uint32_t timeoutMs = 1500 + uint32_t(distanceMm * 10);

  for (;;) {
    service();
    if (buttonDown() || millis() - t0 > timeoutMs) {
      motors::stop();
      return false;
    }
    const float travelled =
        ((encoders::left() - l0) + (encoders::right() - r0)) * 0.5f * MM_PER_TICK;
    float remaining = distanceMm - travelled;

    const uint16_t front = tof::read(tof::FRONT);
    if (useWalls && front != tof::NONE) remaining = min(remaining, float(front) - FRONT_STOP_MM);
    if (!useWalls && front < DIAG_EMERGENCY_STOP_MM) {
      motors::stop();  // about to hit something: the diagonal is off course
      return false;
    }
    if (remaining <= 0) break;

    const float ramp = min(travelled, remaining) * RAMP_PWM_PER_MM;
    const int pwm = min(topPwm, MIN_PWM + int(max(ramp, 0.0f)));
    const float steer = KP_HEADING * (targetYaw - imu::yawDeg) +
                        (useWalls ? KP_WALL * centringError() : 0.0f);
    const int s = constrain(int(steer), -MAX_STEER, MAX_STEER);
    motors::set(pwm - s, pwm + s);  // positive steer = turn left = right wheel faster
    delay(2);
  }
  settle(60);
  return true;
}

// Spins on the spot by `degrees` (positive = left / anticlockwise).
inline bool turn(float degrees) {
  targetYaw += degrees;
  const uint32_t t0 = millis();
  for (;;) {
    service();
    if (buttonDown() || millis() - t0 > 3000) {
      motors::stop();
      return false;
    }
    const float error = targetYaw - imu::yawDeg;
    if (fabsf(error) < TURN_TOLERANCE_DEG) break;
    const int pwm = constrain(int(TURN_KP * fabsf(error)), TURN_MIN_PWM, TURN_MAX_PWM);
    if (error > 0) motors::set(-pwm, pwm);
    else motors::set(pwm, -pwm);
    delay(2);
  }
  settle(80);
  return true;
}

// Carries out one solver move: turn, then drive whole cells.
inline bool execute(const mm::Move& m, int topPwm) {
  bool ok = true;
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
        ok = diagonal ? forward(s.count * DIAG_HALF_MM, min(topPwm, DIAG_PWM), false)
                      : forward(s.count * HALF_CELL_MM, topPwm, true);
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
  tof::waitFresh();
  return Walls{tof::read(tof::LEFT) < WALL_SIDE_MM,
               tof::read(tof::FRONT) < WALL_FRONT_MM,
               tof::read(tof::RIGHT) < WALL_SIDE_MM};
}

}  // namespace motion
