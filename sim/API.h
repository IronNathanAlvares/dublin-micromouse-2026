// API.h - talks to the mms simulator (github.com/mackorone/mms).
// mms sends answers on our stdin and reads commands from our stdout, so never
// print anything else to stdout: use std::cerr for logging.
#pragma once

#include <string>

class API {
 public:
  static int mazeWidth();
  static int mazeHeight();

  static bool wallFront();
  static bool wallRight();
  static bool wallLeft();

  static bool moveForward(int cells = 1);  // false if the mouse crashed
  static bool moveForwardHalf(int halfSteps = 1);
  static void turnRight();
  static void turnLeft();
  static void turnRight45();
  static void turnLeft45();

  static void setWall(int x, int y, char direction);  // direction: n e s w
  static void clearWall(int x, int y, char direction);

  static void setColor(int x, int y, char color);
  static void clearColor(int x, int y);
  static void clearAllColor();

  static void setText(int x, int y, const std::string& text);
  static void clearText(int x, int y);
  static void clearAllText();

  static bool wasReset();
  static void ackReset();
};
