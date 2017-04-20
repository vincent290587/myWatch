/*
 * FXOS8700.cpp
 *
 *  Created on: 8 mars 2017
 *      Author: Vincent
 */

#include <math.h>
#include "FXOS8700.h"
#include "mathutils.h"

#define NRF_LOG_MODULE_NAME "FXOS"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#define FXOS_RST_PIN 21


FXOS8700::FXOS8700() : MMA8451_n0m1() {
	_mag_x = 0;
	_mag_y = 0;
	_mag_z = 0;
}


bool FXOS8700::init() {

	uint8_t statusCheck;

	if (!this->checkIfPresent(FXOS8700_ADDRESS)) {
		return false;
	}

	// reset
	reset();

	wireReadDataByte(REG_WHO_AM_I, statusCheck);
	NRF_LOG_INFO("Device ID:0x%x\r\n", statusCheck);

	if (statusCheck != FXOS8700CQ_WHOAMI_VAL) {
		return false;
	}

	setStandby();

	// init the accelerometers
	init_bare();

	// init the magnetometer in hybrid mode
	wireReadDataByte(FXOS8700CQ_M_CTRL_REG1, statusCheck);
	statusCheck |= 0x1F;
	wireWriteDataByte(FXOS8700CQ_M_CTRL_REG1, statusCheck);

	wireReadDataByte(FXOS8700CQ_M_CTRL_REG2, statusCheck);
	statusCheck |= 0x20;
	wireWriteDataByte(FXOS8700CQ_M_CTRL_REG2, statusCheck);

	// Global shared ODR
	setODR(ODR_800Hz);

	// go !!
	setActive();

	return true;
}


void FXOS8700::update() {

	MMA8451_n0m1::update();

	uint8_t val[7];

	// read magnetometer
	if (!wireReadDataBlock(FXOS8700_M_DR_STATUS, val, 7)) {
		return;
	}

	float factor = 1 / 10.0;

	int16_t x = (int16_t)((val[1] << 8) | val[2]);
	int16_t y = (int16_t)((val[3] << 8) | val[4]);
	int16_t z = (int16_t)((val[5] << 8) | val[6]);

	_mag_x = (float)x * factor;
	_mag_y = (float)y * factor;
	_mag_z = (float)z * factor;

	return;
}


float FXOS8700::computeNorthDirection() {

	float angle;

	angle = (atan2 (_mag_x, _mag_y) + PI) * 180 / PI;

	return angle;

}


/*******************************************************************************
 * I2C Reads and Writes
 ******************************************************************************/
/**
 * @brief Writes a single byte to the I2C device and specified register
 *
 * @param[in] reg the register in the I2C device to write to
 * @param[in] val the 1-byte value to write to the I2C device
 * @return True if successful write operation. False otherwise.
 */
bool FXOS8700::wireWriteDataByte(uint8_t reg, uint8_t val) {

	if (!i2c_write_reg_8(FXOS8700_ADDRESS, reg, val)) {
		return false;
	}

	return true;
}

/**
 * @brief Reads a single byte from the I2C device and specified register
 *
 * @param[in] reg the register to read from
 * @param[out] the value returned from the register
 * @return True if successful read operation. False otherwise.
 */
bool FXOS8700::wireReadDataByte(uint8_t reg, uint8_t &val) {

	if (!i2c_write8(FXOS8700_ADDRESS, reg)) {
		//return false;
	}

	// repeated start
	if (!i2c_read8(FXOS8700_ADDRESS, &val)) {
		return false;
	}

	return true;
}

/**
 * @brief Reads a block (array) of bytes from the I2C device and register
 *
 * @param[in] reg the register to read from
 * @param[out] val pointer to the beginning of the data
 * @param[in] len number of bytes to read
 * @return Number of bytes read. -1 on read error.
 */
int FXOS8700::wireReadDataBlock(uint8_t reg, uint8_t *val, unsigned int len) {

	if (!i2c_write8(FXOS8700_ADDRESS, reg)) {
		//return false;
	}

	// repeated start
	if (!i2c_read_n(FXOS8700_ADDRESS, val, len)) {
		return false;
	}

	return true;
}
