// config.h - every pin number and tuning value in one place.
//
// Pins follow the event wiring map from the Debug Kit (steps 2-8 and the
// encoder tool). If your team wired something differently, change it HERE.
// Values marked CALIBRATE are guesses until you measure your own mouse
// (see README.md -> "Calibrating on the bench").
#pragma once

#include <stdint.h>

// ============================ Pins (ESP32-C6) ============================
// I2C bus shared by the IMU and the three distance sensors.
constexpr uint8_t PIN_SDA = 6;
constexpr uint8_t PIN_SCL = 7;

// VL53L0X XSHUT lines and the address each sensor is given at boot.
constexpr uint8_t PIN_XSHUT_LEFT  = 18;
constexpr uint8_t PIN_XSHUT_FRONT = 19;
constexpr uint8_t PIN_XSHUT_RIGHT = 20;
constexpr uint8_t TOF_ADDR_LEFT  = 0x30;
constexpr uint8_t TOF_ADDR_FRONT = 0x31;
constexpr uint8_t TOF_ADDR_RIGHT = 0x29;  // keeps the default, as in Debug Kit step 5

// DRI0044 motor driver. Motor A = left wheel, Motor B = right wheel.
constexpr uint8_t PIN_DIR_LEFT  = 0;
constexpr uint8_t PIN_PWM_LEFT  = 2;
constexpr uint8_t PIN_DIR_RIGHT = 3;
constexpr uint8_t PIN_PWM_RIGHT = 10;  // NOT GPIO5, that's a strapping pin

// Hall encoders. Left = Motor A, same as the Debug Kit encoder tool.
constexpr uint8_t PIN_ENC_LEFT_A = 21;
constexpr uint8_t PIN_ENC_LEFT_B = 22;
// TBC: the event map doesn't list Motor B's encoder. GPIO23 and GPIO11 are
// free on the DevKitC-1 and aren't strapping/USB pins. Match your wiring!
constexpr uint8_t PIN_ENC_RIGHT_A = 23;
constexpr uint8_t PIN_ENC_RIGHT_B = 11;

constexpr uint8_t PIN_BUTTON = 9;  // the board's BOOT button, LOW when pressed
constexpr uint8_t PIN_RGB = 8;     // onboard RGB LED

// ================================ Maze ==================================
constexpr int MAZE_WIDTH = 16;   // set to the practice maze size when testing on a mini maze
constexpr int MAZE_HEIGHT = 16;
constexpr float CELL_MM = 180.0f;
constexpr int EXPLORE_ROUNDS = 2;  // centre-and-back trips before speed runs (sim: 2 = optimal 99%)
constexpr int SPEED_RUNS = 3;      // speed runs per button press

// ============================== Motors ==================================
constexpr uint32_t PWM_FREQ_HZ = 20000;
constexpr uint8_t PWM_BITS = 8;  // duty 0..255
// 170/255 of a 9 V battery is about 6 V, the N20 motors' rating. The event
// says keep this cap. Every PWM value below gets clamped to it.
constexpr int PWM_LIMIT = 170;

// Flip these if a wheel spins backwards on the 'p' test (instead of rewiring).
constexpr bool MOTOR_LEFT_REVERSED = false;
constexpr bool MOTOR_RIGHT_REVERSED = false;
// Flip these if a wheel's count goes DOWN while it drives forward on 'p'.
constexpr bool ENC_LEFT_REVERSED = false;
constexpr bool ENC_RIGHT_REVERSED = true;  // mirrored motor, usually counts backwards. CALIBRATE

// ============================= Geometry =================================
constexpr float WHEEL_DIAMETER_MM = 34.0f;    // CALIBRATE: measure your wheel
// CALIBRATE: turn a wheel exactly one revolution by hand and read the 's'
// output. We count both edges of channel A, so this is 2x the encoder tool's number.
constexpr float TICKS_PER_WHEEL_REV = 420.0f;
constexpr float MM_PER_TICK = 3.14159265f * WHEEL_DIAMETER_MM / TICKS_PER_WHEEL_REV;

// ======================= Wall sensing (VL53L0X) =========================
// Place the mouse in the middle of a cell and read 's' with and without walls.
constexpr uint16_t WALL_SIDE_MM = 120;    // CALIBRATE: side reading below this = wall
constexpr uint16_t WALL_FRONT_MM = 140;   // CALIBRATE: front reading below this = wall
constexpr uint16_t SIDE_CENTRE_MM = 55;   // CALIBRATE: side reading when centred in a cell
constexpr uint16_t FRONT_STOP_MM = 50;    // CALIBRATE: front reading when centred, facing a wall
constexpr uint16_t TOF_MAX_MM = 1200;     // anything further is treated as "nothing there"

// =============================== IMU ====================================
// +1 if turning the mouse LEFT (anticlockwise from above) makes 's' show a
// growing yaw, -1 if it shrinks.
constexpr float GYRO_Z_SIGN = 1.0f;  // CALIBRATE

// ========================= Speeds and gains =============================
constexpr int EXPLORE_PWM = 90;     // top speed while mapping
constexpr int RUN_PWM = 130;        // top speed on speed runs (cap is PWM_LIMIT)
constexpr int RUN_PWM_STEP = 0;     // add this much after each successful speed run
constexpr int MIN_PWM = 45;         // CALIBRATE: lowest PWM that still moves the mouse ('p')
constexpr float RAMP_PWM_PER_MM = 1.0f;  // acceleration/braking ramp

constexpr float KP_HEADING = 3.0f;  // PWM per degree off course
constexpr float KP_WALL = 0.6f;     // PWM per mm off centre between walls
constexpr int MAX_STEER = 40;

// ============================= Diagonals ================================
// Speed runs cut 45 degree diagonals through staircases when that's faster
// (DiagonalPlanner.h). On a diagonal the side sensors are useless, so the
// mouse steers on gyro + encoders only: get the geometry values above right
// first, and try the 'D' bench test before enabling it for real runs.
// The mouse spins between two posts and passes post corners ~64 mm from its
// centre line, so it must be under ~110 mm wide (measure yours!).
constexpr bool USE_DIAGONALS = true;
constexpr bool EXPLORE_FOR_DIAGONALS = true;  // look at unexplored cells a shortcut might use
constexpr int DIAG_PWM = 100;        // top speed on diagonals (slower: no wall feedback)
constexpr uint16_t DIAG_EMERGENCY_STOP_MM = 25;  // front reading that aborts a diagonal
constexpr float HALF_CELL_MM = CELL_MM / 2;
constexpr float DIAG_HALF_MM = CELL_MM * 0.70710678f;  // one edge middle to the next
// CALIBRATE: time of each motion on YOUR mouse, to decide when a diagonal is
// worth it. Time a straight cell, a 45 degree turn and a diagonal step with
// a stopwatch and put in any consistent unit (these are the mms simulator's).
constexpr float DIAG_TIME_HALF_CELL = 50.0f;
constexpr float DIAG_TIME_DIAG_HALF = 70.71f;
constexpr float DIAG_TIME_TURN45 = 16.66f;

constexpr float TURN_KP = 2.0f;     // PWM per degree still to turn
constexpr int TURN_MIN_PWM = 55;
constexpr int TURN_MAX_PWM = 110;
constexpr float TURN_TOLERANCE_DEG = 2.0f;
