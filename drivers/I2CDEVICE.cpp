/*
 * I2CDEVICE.cpp
 *
 *  Created on: 2 mars 2017
 *      Author: Vincent
 */

#include "I2CDEVICE.h"

#define NRF_LOG_MODULE_NAME "I2C"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

I2C_Device::I2C_Device() {
	_in_error = false;
}

bool I2C_Device::isPresent(uint8_t address) {
	return _in_error;
}


bool I2C_Device::checkIfPresent(uint8_t address) {
	_in_error = i2c_device_present(address);
	return _in_error;
}

