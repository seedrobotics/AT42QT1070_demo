# AT42QT1070_demo
Arduino Sketch to Test and Demonstrate the AT42QT1070 capacitive touch detection chip.

This sketch was written for Teensy 3.2 connected to AT42QT1070 chip over I2C.

The skecth includes a C++ class that encapsulates most of the communication with the chip.

It uses a teensy-specific library for I2C communication (i2c_t3) although modifying it to run on a normal Arduino would only require users to:
- Change the include of "i2c3_t3.h" to "Wire.h"
- Change references to type "i2c_t3" to "Wire"


