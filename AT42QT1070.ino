/* Demonstration code for AT42QT sensors.
	First sample, based on AT42QT1070 */

/** Copyright 2020 Seed Robotics Ltd

	Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
	documentation files (the "Software"), to deal in the Software without restriction, including without limitation
	the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software,
	and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO
	THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
	OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
	TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE. */

#include "Arduino.h"
#include "AT42QT.h"

// This skecth requires I2C to communicate with the chip

//***-->> This sketch was originally written for Teensy 3.2 and thus it is written to use the i2c_t3 library
// if using other boards remove the reference to I2c_t3.h and replace it with an #include "Wire.h" (this is untested though)
// you will also need to rename the types i2c_t3 to Wire or equivalent
#include "i2c_t3.h"

SeedRobotics_AT42QT cap;

void setup() {

	Serial.begin(9600);

	delay(2000);

	Serial.println("AT42QT test!");

	// Initialize the sensor, if using i2c you can pass in the i2c address
	// if (!cap.begin(0x28)) {
	if (!cap.begin(AT42QT1070_DEVICE_ADDR, &Wire1)) {
		Serial.println("AT42QT not found");
		while (1);
	}
	Serial.println("AT42QT found!");

	cap.guard_channel_disable();

	// disable recallibration after a long touch
	cap.set_max_on_duration(0);

	// increase avg factor on channels
	cap.set_avg_factor(0, 32);
	cap.set_avg_factor(1, 32);

	// force recallibration
	cap.recallibrate();
}

// Add the main program code into the continuous loop() function
void loop()
{


	while (1) {
		uint8_t pressed_status = cap.get_touchedkeys_bitmap();
		Serial.print("Keys pressed: (ch.nr): ");

		for (byte b = 0; b < 7; b++) {
			if (pressed_status & ( (uint8_t) pow(2, (uint16_t)b) ) ) Serial.printf("%d ", b);
		}
		Serial.println();


		for (byte b = 0; b < 7; b++) {
			Serial.printf("[%d] Signal %d\tReference %d\tNeg Thr. %d\tAvg Factor %d\n",
				b, cap.get_key_signal(b), cap.get_key_reference(b), cap.get_neg_threshold(b), cap.get_avg_factor(b)); /**/
		}
		Serial.println();
		Serial.println();

		// adjust the delay as needed; you should be able to run the loop without any delay
		delay(1500);
	}	
}
