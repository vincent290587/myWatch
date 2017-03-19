/*
 * STC3100.cpp
 *
 *  Created on: 1 mars 2017
 *      Author: Vincent
 */


#include "Arduino.h"

#include "STC3100.h"
#include "I2C.h"

#define NRF_LOG_MODULE_NAME "STC"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"



/*=========================================================================*/

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/

#define REG_MODE                0
#define REG_CONTROL             1
#define REG_CHARGE_LOW          2
#define REG_CHARGE_HIGH         3
#define REG_COUNTER_LOW         4
#define REG_COUNTER_HIGH        5
#define REG_CURRENT_LOW         6
#define REG_CURRENT_HIGH        7
#define REG_VOLTAGE_LOW         8
#define REG_VOLTAGE_HIGH        9
#define REG_TEMPERATURE_LOW     10
#define REG_TEMPERATURE_HIGH    11
#define REG_DEVICE_ID           24
/*=========================================================================*/


/***************************************************************************
 CONSTRUCTOR
 ***************************************************************************/

STC3100::STC3100(int32_t sensorID) : I2C_Device () {
	_sensorID = sensorID;
}

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Setups the HW
 */
/**************************************************************************/
bool STC3100::init(uint32_t r_sens, stc3100_res_t res) {


	if (!this->checkIfPresent(STC3100_ADDRESS)) {
		return false;
	}

	_r_sens = r_sens;

    // reset
	writeCommand(REG_CONTROL, STC_RESET);
	delay(1);

	// read device ID
	i2c_read_reg_8(STC3100_ADDRESS, REG_DEVICE_ID, &_deviceID);
	NRF_LOG_INFO("Device ID: %x\r\n", _deviceID);

	/* Set the mode indicator */
	_stc3100Mode = 0;
	_stc3100Mode |= MODE_RUN;
	_stc3100Mode |= res;

	// set mode
	writeCommand(REG_MODE, _stc3100Mode);

	return true;
}



/**************************************************************************/
/*!
    @brief  Reads the sensor
 */
/**************************************************************************/
bool STC3100::refresh()
{
	readChip();

	computeVoltage ();
	computeCharge  ();
	computeCurrent ();
	computeTemp    ();

	return true;
}


void STC3100::computeCounter()
{
	uint8_t tl=_stc_data.CounterLow, th=_stc_data.CounterHigh;
	uint32_t t;
	int val;

	t = th;
	t <<= 8;
	val = (t & 0xFF00) | tl;
	_counter = (float) val;
}

void STC3100::computeVoltage()
{

	uint8_t tl=_stc_data.VoltageLow, th=_stc_data.VoltageHigh;
	uint32_t t;
	int val;

	t = th;
	t <<= 8;
	val = (t & 0xFF00) | tl;

	_voltage = (float)  val * 0.00244;

}

void STC3100::computeCharge()
{

	uint8_t tl=_stc_data.ChargeLow, th=_stc_data.ChargeHigh;
	uint32_t t;
	int val;

	t = th;
	t <<= 8;
	val = (t & 0xFF00) | tl;
	_charge = ((float) val * 6.7 / _r_sens);

}

void STC3100::computeCurrent()
{
	uint8_t tl=_stc_data.CurrentLow, th=_stc_data.CurrentHigh;
	float val;

	NRF_LOG_INFO("Current L=0x%x H=0x%x\r\n", tl, th);

	val = compute2Complement(th, tl);
	_current = (val * 11.77 / _r_sens);

}


void STC3100::computeTemp()
{
	uint8_t tl=_stc_data.TemperatureLow, th=_stc_data.TemperatureHigh;
	uint32_t t;
	int val;

	t = th;
	t <<= 8;
	val = (t & 0xFF00) | tl;

	_temperature = ((float) val * 0.125);

}





/***************************************************************************
 PRIVATE FUNCTIONS
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Writes an 8 bit value over I2C
 */
/**************************************************************************/
void STC3100::writeCommand(uint8_t reg, uint8_t value)
{
	i2c_write_reg_8(STC3100_ADDRESS, reg, value);
}

void STC3100::readChip()
{

	i2c_read_reg_n(STC3100_ADDRESS, REG_CHARGE_LOW, _stc_data.array, 10);

}
