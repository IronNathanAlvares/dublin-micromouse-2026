# Pin map: ESP32-C6-DevKitC-1

From the event Debug Kit (steps 2–8 and the encoder tool). The firmware's
`firmware/micromouse/config.h` uses exactly these numbers. If you change the
wiring, change both.

| Signal | GPIO | Device pin | Notes |
|---|---|---|---|
| I2C SDA | 6 | SDA on IMU + all 3 ToF | shared bus, 3.3 V |
| I2C SCL | 7 | SCL on IMU + all 3 ToF | shared bus, 3.3 V |
| Left ToF XSHUT | 18 | XSHUT | gets address 0x30 at boot |
| Front ToF XSHUT | 19 | XSHUT | gets address 0x31 at boot |
| Right ToF XSHUT | 20 | XSHUT | keeps default 0x29 |
| IMU (MPU-6050) | – | – | address 0x68; leave XDA, XCL, AD0, INT unconnected |
| Left motor DIR | 0 | DRI0044 DIR1 | Motor A |
| Left motor PWM | 2 | DRI0044 PWM1 | Motor A |
| Right motor DIR | 3 | DRI0044 DIR2 | Motor B |
| Right motor PWM | 10 | DRI0044 PWM2 | not GPIO5 (strapping pin) |
| Left encoder A / B | 21 / 22 | encoder C1 / C2 | encoder VCC on 3V3, never the battery |
| Right encoder A / B | **23 / 11 (TBC)** | encoder C1 / C2 | not in the event map, so confirm with your wiring |
| Start button | 9 | onboard BOOT button | LOW when pressed |
| RGB LED | 8 | onboard | |

Power: sensors and encoders on 3V3. Driver VM on the motor battery. Battery −,
driver GND and ESP32 GND joined. PWM is capped at 170/255 (≈6 V from 9 V).
