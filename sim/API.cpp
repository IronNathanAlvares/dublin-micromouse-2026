#include "API.h"

#include <iostream>

namespace {

std::string readLine() {
  std::string line;
  std::getline(std::cin, line);
  if (!line.empty() && line.back() == '\r') line.pop_back();
  return line;
}

// Sends a command and waits for mms to answer it.
std::string ask(const std::string& command) {
  std::cout << command << std::endl;
  return readLine();
}

void tell(const std::string& command) { std::cout << command << std::endl; }

std::string cell(int x, int y) { return std::to_string(x) + " " + std::to_string(y); }

}  // namespace

int API::mazeWidth() { return std::stoi(ask("mazeWidth")); }
int API::mazeHeight() { return std::stoi(ask("mazeHeight")); }

bool API::wallFront() { return ask("wallFront") == "true"; }
bool API::wallRight() { return ask("wallRight") == "true"; }
bool API::wallLeft() { return ask("wallLeft") == "true"; }

bool API::moveForward(int cells) {
  return ask("moveForward " + std::to_string(cells)) == "ack";
}
bool API::moveForwardHalf(int halfSteps) {
  return ask("moveForwardHalf " + std::to_string(halfSteps)) == "ack";
}
void API::turnRight() { ask("turnRight"); }
void API::turnLeft() { ask("turnLeft"); }
void API::turnRight45() { ask("turnRight45"); }
void API::turnLeft45() { ask("turnLeft45"); }

void API::setWall(int x, int y, char direction) {
  tell("setWall " + cell(x, y) + " " + direction);
}
void API::clearWall(int x, int y, char direction) {
  tell("clearWall " + cell(x, y) + " " + direction);
}

void API::setColor(int x, int y, char color) { tell("setColor " + cell(x, y) + " " + color); }
void API::clearColor(int x, int y) { tell("clearColor " + cell(x, y)); }
void API::clearAllColor() { tell("clearAllColor"); }

void API::setText(int x, int y, const std::string& text) {
  tell("setText " + cell(x, y) + " " + text);
}
void API::clearText(int x, int y) { tell("clearText " + cell(x, y)); }
void API::clearAllText() { tell("clearAllText"); }

bool API::wasReset() { return ask("wasReset") == "true"; }
void API::ackReset() { ask("ackReset"); }
