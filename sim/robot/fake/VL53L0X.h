// VL53L0X.h (fake) - same interface as the Pololu library calls the firmware
// makes; ranges come from ray-casting the virtual maze.
#pragma once

#include "Arduino.h"

class VL53L0X {
 public:
  enum regAddr { RESULT_INTERRUPT_STATUS = 0x13 };

  void setAddress(uint8_t address) { address_ = address; }
  uint8_t getAddress() { return address_; }
  bool init(bool = true) { sensor_ = simhw::tofClaim(); return sensor_ >= 0; }
  void setTimeout(uint16_t) {}
  bool setMeasurementTimingBudget(uint32_t us) { budgetUs_ = us; return true; }
  void startContinuous(uint32_t = 0) { lastUs_ = simhw::nowUs(); }
  uint8_t readReg(uint8_t) {  // only RESULT_INTERRUPT_STATUS is used
    simhw::advanceUs(100);
    return simhw::nowUs() - lastUs_ >= budgetUs_ ? 0x07 : 0x00;
  }
  uint16_t readRangeContinuousMillimeters() {
    simhw::advanceUs(200);
    lastUs_ = simhw::nowUs();
    return simhw::tofRange(sensor_);
  }
  bool timeoutOccurred() { return false; }

 private:
  uint8_t address_ = 0x29;
  int sensor_ = -1;
  uint32_t budgetUs_ = 33000;
  uint64_t lastUs_ = 0;
};
