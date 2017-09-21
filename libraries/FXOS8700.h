/*
 * FXOS8700.h
 *
 *  Created on: 8 mars 2017
 *      Author: Vincent
 */

#ifndef LIBRARIES_FXOS8700_H_
#define LIBRARIES_FXOS8700_H_

#include "MMA8451.h"

#define FXOS8700_ADDRESS 0x1D

#define FXOS8700_M_DR_STATUS       0x32 // Magnetic data ready
#define FXOS8700_M_OUT_X_MSB       0x33 // MSB of 16-bit magnetic data for X-axis
#define FXOS8700_M_OUT_X_LSB       0x34 // LSB of 16-bit magnetic data for X-axis
#define FXOS8700_M_OUT_Y_MSB       0x35 // MSB of 16-bit magnetic data for Y-axis
#define FXOS8700_M_OUT_Y_LSB       0x36 // LSB of 16-bit magnetic data for Y-axis
#define FXOS8700_M_OUT_Z_MSB       0x37 // MSB of 16-bit magnetic data for Z-axis
#define FXOS8700_M_OUT_Z_LSB       0x38 // LSB of 16-bit magnetic data for Z-axis

#define FXOS8700CQ_STATUS 0x00
#define FXOS8700CQ_WHOAMI 0x00
#define FXOS8700CQ_XYZ_DATA_CFG 0x0E
#define FXOS8700CQ_CTRL_REG1 0x2A
#define FXOS8700CQ_M_CTRL_REG1 0x5B
#define FXOS8700CQ_M_CTRL_REG2 0x5C
#define FXOS8700CQ_WHOAMI_VAL 0xC7


class FXOS8700 : public MMA8451_n0m1 {
public:
	FXOS8700();
	bool init();
	void update();
	float computeNorthDirection();

	float getMagX() const {
		return _mag_x;
	}

	float getMagY() const {
		return _mag_y;
	}

	float getMagZ() const {
		return _mag_z;
	}

private:
	float _mag_x;
	float _mag_y;
	float _mag_z;

	bool isInit;

	float _min_mag_x;
	float _min_mag_y;
	float _min_mag_z;
	float _max_mag_x;
	float _max_mag_y;
	float _max_mag_z;

	void resetMag() {isInit=false;}

	bool wireWriteDataByte(uint8_t reg, uint8_t val);
	bool wireReadDataByte(uint8_t reg, uint8_t &val);
	int wireReadDataBlock(uint8_t reg, uint8_t *val, unsigned int len);
};

#endif /* LIBRARIES_FXOS8700_H_ */
