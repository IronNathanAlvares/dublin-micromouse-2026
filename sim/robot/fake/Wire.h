// Wire.h (fake) - I2C bus routed to the virtual robot's MPU-6050.
#pragma once

#include "Arduino.h"

class TwoWire {
 public:
  bool begin(int, int) { return true; }
  void end() {}
  void setClock(uint32_t) {}
  void beginTransmission(uint8_t address) { address_ = address; txLen_ = 0; }
  size_t write(uint8_t b) { if (txLen_ < 32) tx_[txLen_++] = b; return 1; }
  uint8_t endTransmission(bool = true) { return uint8_t(simhw::i2cWrite(address_, tx_, txLen_)); }
  uint8_t requestFrom(uint8_t address, uint8_t n) {
    rxLen_ = simhw::i2cRead(address, rx_, n > 32 ? 32 : n);
    rxPos_ = 0;
    return uint8_t(rxLen_);
  }
  int read() { return rxPos_ < rxLen_ ? rx_[rxPos_++] : -1; }

 private:
  uint8_t address_ = 0, tx_[32] = {}, rx_[32] = {};
  int txLen_ = 0, rxLen_ = 0, rxPos_ = 0;
};
inline TwoWire Wire;
