/*
 * VEML6075.cpp
 *
 *  Created on: 28 févr. 2017
 *      Author: Vincent
 */

#include "I2C.h"
#include "Arduino.h"

#define NRF_LOG_MODULE_NAME "VEML"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include "VEML6075.h"


#define VEML6075_ADDR  0x10
#define VEML6075_DEVID 0x26



VEML6075::VEML6075() : I2C_Device () {

	// Despite the datasheet saying this isn't the default on startup, it appears
	// like it is. So tell the thing to actually start gathering data.
	this->config = 0;
	this->config |= VEML6075_CONF_PW_ON;

	// App note only provided math for this one...
	this->config |= VEML6075_CONF_IT_100MS;
}

bool VEML6075::init() {

	if (this->getDevID() != VEML6075_DEVID) {
		NRF_LOG_ERROR("Wrong device ID\r\n");
		return false;
	}

	// Write config to make sure device is enabled
	this->write16(VEML6075_REG_CONF, this->config);

	return true;
}

// Poll sensor for latest values and cache them
void VEML6075::poll() {
	this->raw_uva = this->read16(VEML6075_REG_UVA);
	this->raw_uvb = this->read16(VEML6075_REG_UVB);
	this->raw_dark = this->read16(VEML6075_REG_DUMMY);
	this->raw_vis = this->read16(VEML6075_REG_UVCOMP1);
	this->raw_ir = this->read16(VEML6075_REG_UVCOMP2);
}

uint16_t VEML6075::getRawUVA() {
	return this->raw_uva;
}

uint16_t VEML6075::getRawUVB() {
	return this->raw_uvb;
}

uint16_t VEML6075::getRawDark() {
	return this->raw_dark;
}

uint16_t VEML6075::getRawVisComp() {
	return this->raw_vis;
}

uint16_t VEML6075::getRawIRComp() {
	return this->raw_ir;
}

uint16_t VEML6075::getDevID() {
	return this->read16(VEML6075_REG_DEVID);
}

float VEML6075::getUVA() {
	float comp_vis = this->raw_vis - this->raw_dark;
	float comp_ir = this->raw_ir - this->raw_dark;
	float comp_uva = this->raw_uva - this->raw_dark;

	comp_uva -= VEML6075_UVI_UVA_VIS_COEFF * comp_vis;
	comp_uva -= VEML6075_UVI_UVA_IR_COEFF * comp_ir;

	return comp_uva;
}

float VEML6075::getUVB() {
	float comp_vis = this->raw_vis - this->raw_dark;
	float comp_ir = this->raw_ir - this->raw_dark;
	float comp_uvb = this->raw_uvb - this->raw_dark;

	comp_uvb -= VEML6075_UVI_UVB_VIS_COEFF * comp_vis;
	comp_uvb -= VEML6075_UVI_UVB_IR_COEFF * comp_ir;

	return comp_uvb;
}

float VEML6075::getUVIndex() {
	float uva_weighted = this->getUVA() * VEML6075_UVI_UVA_RESPONSE;
	float uvb_weighted = this->getUVB() * VEML6075_UVI_UVB_RESPONSE;
	return (uva_weighted + uvb_weighted) / 2.0;
}

/////// I2C functions  ////////

uint16_t VEML6075::read16(uint8_t reg) {

	if (i2c_write8(VEML6075_ADDR, reg)) {
		//NRF_LOG_ERROR("Error on I2C\r\n");
		//return false;
	}

	// read from SPI
	uint8_t raw_data[2];
	if (!i2c_read_n(VEML6075_ADDR, raw_data, 2)) {
		return 0xFFFF;
	}

	return (raw_data[1] << 8) | raw_data[0];
}

void VEML6075::write16(uint8_t reg, uint16_t data) {

	// reg LSB MSB
	uint8_t raw_data[2] = {(uint8_t)(0xFF & data), (uint8_t)(0xFF & (data >> 8))};

	(void)i2c_write_reg_n(VEML6075_ADDR, reg, raw_data, 2);

}
