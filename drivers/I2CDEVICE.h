/*
 * I2CDEVICE.h
 *
 *  Created on: 2 mars 2017
 *      Author: Vincent
 */

#ifndef DRIVERS_I2CDEVICE_H_
#define DRIVERS_I2CDEVICE_H_

#include "I2C.h"

class I2C_Device {
public:
	I2C_Device();
	bool isPresent(uint8_t address);
	bool checkIfPresent(uint8_t address);
private:
	bool _in_error;
};

#endif /* DRIVERS_I2CDEVICE_H_ */
