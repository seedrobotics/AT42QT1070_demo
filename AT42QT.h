#ifndef AT42QT1070_H_
#define AT42QT1070_H_

// AT42QT.h
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
#include "i2c_t3.h"

/*** Register definitions for AT42QT chip (taken from AT42QT1107 datasheet)
	https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-9596-AT42-QTouch-BSW-AT42QT1070_Datasheet.pdf

*/
#define AT42QT1070_CHIP_ID 0x0
#define AT42QT1070_FIRMWARE_VERSION 0x1
#define AT42QT1070_DETECTION_STATUS 0x2
#define AT42QT1070_KEY_STATUS 0x3
#define AT42QT1070_KEY_SIGNAL_0_HIGH 0x4
#define AT42QT1070_KEY_SIGNAL_0_LOW 0x5
#define AT42QT1070_KEY_SIGNAL_1_HIGH 0x6
#define AT42QT1070_KEY_SIGNAL_1_LOW 0x7
#define AT42QT1070_KEY_SIGNAL_2_HIGH 0x8
#define AT42QT1070_KEY_SIGNAL_2_LOW 0x9
#define AT42QT1070_KEY_SIGNAL_3_HIGH 0xA
#define AT42QT1070_KEY_SIGNAL_3_LOW 0xB
#define AT42QT1070_KEY_SIGNAL_4_HIGH 0xC
#define AT42QT1070_KEY_SIGNAL_4_LOW 0xD
#define AT42QT1070_KEY_SIGNAL_5_HIGH 0xE
#define AT42QT1070_KEY_SIGNAL_5_LOW 0xF
#define AT42QT1070_KEY_SIGNAL_6_HIGH 0x10
#define AT42QT1070_KEY_SIGNAL_6_LOW 0x11
#define AT42QT1070_REFERENCE_DATA_0_HIGH 0x12
#define AT42QT1070_REFERENCE_DATA_0_LOW 0x13
#define AT42QT1070_REFERENCE_DATA_1_HIGH 0x14
#define AT42QT1070_REFERENCE_DATA_1_LOW 0x15
#define AT42QT1070_REFERENCE_DATA_2_HIGH 0x16
#define AT42QT1070_REFERENCE_DATA_2_LOW 0x17
#define AT42QT1070_REFERENCE_DATA_3_HIGH 0x18
#define AT42QT1070_REFERENCE_DATA_3_LOW 0x19
#define AT42QT1070_REFERENCE_DATA_4_HIGH 0x1A
#define AT42QT1070_REFERENCE_DATA_4_LOW 0x1B
#define AT42QT1070_REFERENCE_DATA_5_HIGH 0x1C
#define AT42QT1070_REFERENCE_DATA_5_LOW 0x1D
#define AT42QT1070_REFERENCE_DATA_6_HIGH 0x1E
#define AT42QT1070_REFERENCE_DATA_6_LOW 0x1F
#define AT42QT1070_NTHR_KEY_0 0x20
#define AT42QT1070_NTHR_KEY_1 0x21
#define AT42QT1070_NTHR_KEY_2 0x22
#define AT42QT1070_NTHR_KEY_3 0x23
#define AT42QT1070_NTHR_KEY_4 0x24
#define AT42QT1070_NTHR_KEY_5 0x25
#define AT42QT1070_NTHR_KEY_6 0x26
#define AT42QT1070_AVE_AKS_KEY_0 0x27
#define AT42QT1070_AVE_AKS_KEY_1 0x28
#define AT42QT1070_AVE_AKS_KEY_2 0x29
#define AT42QT1070_AVE_AKS_KEY_3 0x2A
#define AT42QT1070_AVE_AKS_KEY_4 0x2B
#define AT42QT1070_AVE_AKS_KEY_5 0x2C
#define AT42QT1070_AVE_AKS_KEY_6 0x2D
#define AT42QT1070_DI_KEY_0 0x2E
#define AT42QT1070_DI_KEY_1 0x2F
#define AT42QT1070_DI_KEY_2 0x30
#define AT42QT1070_DI_KEY_3 0x31
#define AT42QT1070_DI_KEY_4 0x32
#define AT42QT1070_DI_KEY_5 0x33
#define AT42QT1070_DI_KEY_6 0x34
#define AT42QT1070_FO_MO_GUARDCH 0x35
#define AT42QT1070_LP_MODE 0x36
#define AT42QT1070_MAXON_DURATION 0x37
#define AT42QT1070_CALIBRATE 0x38
#define AT42QT1070_RESET 0x39


#define AT42QT1070_DEVICE_ADDR 0x1B

class SeedRobotics_AT42QT {
public:
	boolean begin(uint8_t i2caddr = AT42QT1070_DEVICE_ADDR, i2c_t3 *theWire = &Wire);
	
	void soft_reset();

	uint8_t get_touchedkeys_bitmap();
	uint8_t get_detection_flags();
	uint16_t get_key_signal(uint8_t ch_nr);
	uint16_t get_key_reference(uint8_t ch_nr);
	
	uint16_t get_neg_threshold(uint8_t ch_nr);
	void set_neg_threshold(uint8_t ch_nr, uint8_t threshold);

	uint16_t get_avg_factor(uint8_t ch_nr);
	void set_avg_factor(uint8_t ch_nr, uint8_t factor);

	uint8_t get_aks_group(uint8_t ch_nr);
	void set_aks_group(uint8_t ch_nr, uint8_t group);

	uint8_t get_di_count(uint8_t ch_nr);
	void set_di_count(uint8_t ch_nr, uint8_t count);

	uint8_t get_max_on_duration();
	void set_max_on_duration(uint8_t value);


	void guard_channel_disable();
	void guard_channel_set_ch(uint8_t ch_nr);

	void recallibrate();

	void i2cwrite(uint8_t x);
	uint8_t readRegister(uint8_t reg);
	uint16_t readRegisters_WORD(uint8_t msb_reg, uint8_t lsb_reg);
	void writeRegister(uint8_t reg, uint8_t value);

private:
	int8_t _i2caddr;
	i2c_t3 *_wire;
	bool initialized_ = false;
};


#endif

