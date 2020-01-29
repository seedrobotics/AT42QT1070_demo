/** Copyright 2020 Seed Robotics Ltd

	Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
	documentation files (the "Software"), to deal in the Software without restriction, including without limitation
	the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software,
	and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO
	THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
	OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
	TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE. 
	
	
	This software derives or uses part of the code developped by Adafruit for the CAP1188 sensor as per the copyright below

	START COPYRIGHT ADAFRUIT NOTICE
			/***************************************************
				This is a library for the CAP1188 I2C/SPI 8-chan Capacitive Sensor

				Designed specifically to work with the CAP1188 sensor from Adafruit
				----> https://www.adafruit.com/products/1602

				These sensors use I2C/SPI to communicate, 2+ pins are required to
				interface
				Adafruit invests time and resources providing this open source code,
				please support Adafruit and open-source hardware by purchasing
				products from Adafruit!

				Written by Limor Fried/Ladyada for Adafruit Industries.
				BSD license, all text above must be included in any redistribution
			 ***************************************************
	END COPYRIGHT ADAFRUIT NOTICE
*/
#include "AT42QT.h"
#include "i2c_t3.h"

/* Initializes the I2C bus and pings the device to check that it's connected */
boolean SeedRobotics_AT42QT::begin(uint8_t i2caddr, i2c_t3 *theWire) {
	_wire = theWire;
	_i2caddr = i2caddr;

	/* set up I2c
	  we're setting up at 400Khz which is the fatest per the standard.
	  4.7kOHm (or lower) pull ups should be used for this high speed communication.
	  If you have communication issues try runnig at 100Khz to start
	*/
	_wire->begin(I2C_MASTER, 0, I2C_PINS_DEFAULT, I2C_PULLUP_EXT, 400000U, I2C_OP_MODE_ISR);
	_wire->setTimeout(1000);

	// ping the device
	_wire->beginTransmission(i2caddr);
	uint8_t ping_result = _wire->endTransmission();

	if (ping_result != 0) {
		// ping failed; no device found
		return false;
	}

	// reset the chip, in case we're uploading a new sketch without power cycling the chip
	// to ensure a clean start
	soft_reset();

	// Useful debugging info
	Serial.print("Chip ID: 0x");
	Serial.println(readRegister(AT42QT1070_CHIP_ID), HEX);
	Serial.print("Firmware version ID: 0x");
	Serial.println(readRegister(AT42QT1070_FIRMWARE_VERSION), HEX);

	initialized_ = true;

	return initialized_;
}

void SeedRobotics_AT42QT::recallibrate() {
	if (!initialized_) return;
	// force recallibration
	writeRegister(AT42QT1070_CALIBRATE, 1);
	 
	// read the Detection status register until the
	// callibration bit is cleared.
	// this indicated callibration is complete
	uint8_t value;
	while (1) {
		value = get_detection_flags() & 0x80;
		if (value == 0) break;
		delay(100);
	}
}

void SeedRobotics_AT42QT::soft_reset() {
	if (!initialized_) return;

	writeRegister(AT42QT1070_RESET, 1);

	delay(250); // per the datasheet, after a soft reset the chip needs 125ms to initialize; allow some extra room.
}

void SeedRobotics_AT42QT::guard_channel_disable() {
	if (!initialized_) return;
	writeRegister(AT42QT1070_FO_MO_GUARDCH, (readRegister(AT42QT1070_FO_MO_GUARDCH) & 0xF8) | 0x7);
}

void SeedRobotics_AT42QT::guard_channel_set_ch(uint8_t ch_nr) {
	if (!initialized_) return;
	writeRegister(AT42QT1070_FO_MO_GUARDCH, (readRegister(AT42QT1070_FO_MO_GUARDCH) & 0xF8) | (ch_nr & 0x7) );
}


uint16_t SeedRobotics_AT42QT::get_key_signal(uint8_t ch_nr) {
	if (!initialized_) return 0;
	return readRegisters_WORD(AT42QT1070_KEY_SIGNAL_0_HIGH + 2 * ch_nr, AT42QT1070_KEY_SIGNAL_0_LOW + 2 * ch_nr);
}

uint16_t SeedRobotics_AT42QT::get_key_reference(uint8_t ch_nr) {
	if (!initialized_) return 0;
	return readRegisters_WORD(AT42QT1070_REFERENCE_DATA_0_HIGH + 2 * ch_nr, AT42QT1070_REFERENCE_DATA_0_LOW + 2 * ch_nr);
}

uint16_t SeedRobotics_AT42QT::get_neg_threshold(uint8_t ch_nr) {
	if (!initialized_) return 0;
	return readRegister(AT42QT1070_NTHR_KEY_0 + ch_nr);
}
void SeedRobotics_AT42QT::set_neg_threshold(uint8_t ch_nr, uint8_t threshold) {
	if (!initialized_) return;
	writeRegister(AT42QT1070_NTHR_KEY_0 + ch_nr, threshold);
}

uint16_t SeedRobotics_AT42QT::get_avg_factor(uint8_t ch_nr) {
	if (!initialized_) return 0;
	return (readRegister(AT42QT1070_AVE_AKS_KEY_0 + ch_nr)) >> 2;
}
void SeedRobotics_AT42QT::set_avg_factor(uint8_t ch_nr, uint8_t factor) {
	if (!initialized_) return;

	factor = factor & 0x20; // trim to 6 bits to fir the register
	factor = factor << 2;
	factor = factor | get_aks_group(ch_nr); // add in the existing  aks group as they're joined together in the register

	writeRegister(AT42QT1070_AVE_AKS_KEY_0 + ch_nr, factor);
}

uint8_t SeedRobotics_AT42QT::get_aks_group(uint8_t ch_nr) {
	if (!initialized_) return 0;
	return (readRegister(AT42QT1070_AVE_AKS_KEY_0 + ch_nr)) & 0x3;
}
void SeedRobotics_AT42QT::set_aks_group(uint8_t ch_nr, uint8_t group) {
	if (!initialized_) return;

	group = group & 0x3;
	group = (get_avg_factor(ch_nr) << 2) | group;

	writeRegister(AT42QT1070_AVE_AKS_KEY_0 + ch_nr, group);
}

uint8_t SeedRobotics_AT42QT::get_di_count(uint8_t ch_nr) {
	if (!initialized_) return 0;
	return readRegister(AT42QT1070_DI_KEY_0 + ch_nr);
}
void SeedRobotics_AT42QT::set_di_count(uint8_t ch_nr, uint8_t count) {
	if (!initialized_) return;
	writeRegister(AT42QT1070_DI_KEY_0 + ch_nr, count);
}

uint8_t SeedRobotics_AT42QT::get_max_on_duration() {
	if (!initialized_) return 0;
	return readRegister(AT42QT1070_MAXON_DURATION);
}
void SeedRobotics_AT42QT::set_max_on_duration(uint8_t value) {
	if (!initialized_) return;
	writeRegister(AT42QT1070_MAXON_DURATION, value);
}


uint8_t SeedRobotics_AT42QT::get_touchedkeys_bitmap() {
	if (!initialized_) return 0;
	return readRegister(AT42QT1070_KEY_STATUS);
}

uint8_t SeedRobotics_AT42QT::get_detection_flags() {
	if (!initialized_) return 0;
	return readRegister(AT42QT1070_DETECTION_STATUS);
}

/*!
@brief  Abstract away platform differences in Arduino wire library
@param  x
*/
void SeedRobotics_AT42QT::i2cwrite(uint8_t x) {
	_wire->write((uint8_t)x);
}


/*!
*    @brief  Reads from selected register
*    @param  reg
*            register address
*    @return
*/
uint8_t SeedRobotics_AT42QT::readRegister(uint8_t reg) {
	_wire->beginTransmission(_i2caddr);
	i2cwrite(reg);
	_wire->endTransmission();
	_wire->requestFrom(_i2caddr, 1);
	return (_wire->read());
}

uint16_t SeedRobotics_AT42QT::readRegisters_WORD(uint8_t msb_reg, uint8_t lsb_reg) {
	// this function may be optimized if all reads are done with either MSB or LSB first
	// for flexibility sake, we keep this option of specifying and reading the MSB byte and LSB byte
	// individually
	
	uint16_t value;

	value = readRegister(msb_reg);
	value = value << 8;

	value = value + readRegister(lsb_reg);

	return value;
}

/*!
*   @brief  Writes 8-bits to the specified destination register
*   @param  reg
*           register address
*   @param  value
*           value that will be written at selected register
*/
void SeedRobotics_AT42QT::writeRegister(uint8_t reg, uint8_t value) {
	_wire->beginTransmission(_i2caddr);
	i2cwrite((uint8_t)reg);
	i2cwrite((uint8_t)(value));
	_wire->endTransmission();
}


