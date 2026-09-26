// SimHardware.h - the hooks the fake Arduino/library headers call into.
// Implemented by the virtual robot in robot_sim.cpp.
#pragma once

#include <cstdint>

namespace simhw {
uint64_t nowUs();
void advanceUs(uint64_t us);  // moves the virtual clock and the physics on

void pinWrite(uint8_t pin, int level);
int pinRead(uint8_t pin);
void pwmWrite(uint8_t pin, uint32_t duty);
void led(uint8_t r, uint8_t g, uint8_t b);
void attachIsr(uint8_t pin, void (*isr)());

int i2cWrite(uint8_t address, const uint8_t* data, int n);  // 0 = ACK, 2 = no device
int i2cRead(uint8_t address, uint8_t* out, int n);

int tofClaim();                // which ToF (0 left, 1 front, 2 right) is being initialised
uint16_t tofRange(int sensor);  // a fresh reading in mm (8190 = nothing in range)

int serialAvailable();
int serialRead();
void serialWrite(const char* text);
}  // namespace simhw
