/*
 * STC3100.h
 *
 *  Created on: 1 mars 2017
 *      Author: Vincent
 */

#ifndef LIBRARIES_STC3100_H_
#define LIBRARIES_STC3100_H_

#include "I2CDEVICE.h"

/*=========================================================================
    I2C 7-bit ADDRESS
    -----------------------------------------------------------------------*/
#define STC3100_ADDRESS                (0x70)

/*=========================================================================
 MODE SETTINGS
 -----------------------------------------------------------------------*/

#define MODE_RUN                0x10
#define STC_RESET               0x02

typedef enum {
	STC3100_MODE_ULTRAHIGHRES = 0u,
	STC3100_MODE_HIGHRES = 2u,
	STC3100_MODE_STANDARD = 4u
} stc3100_res_t;

typedef union {
	/**Used for simpler readings of the chip.*/
	uint8_t array[10];
	struct {
		uint8_t ChargeLow;
		/**Gas gauge charge data, bits 8-15*/
		uint8_t ChargeHigh;
		/**Number of conversions, bits 0-7*/
		uint8_t CounterLow;
		/**Number of conversions, bits 8-15*/
		uint8_t CounterHigh;
		/**Battery current value, bits 0-7*/
		uint8_t CurrentLow;
		/**Battery current value, bits 8-15*/
		uint8_t CurrentHigh;
		/**Battery voltage value, bits 0-7*/
		uint8_t VoltageLow;
		/**Battery voltage value, bits 8-15*/
		uint8_t VoltageHigh;
		/**Temperature value, bits 0-7*/
		uint8_t TemperatureLow;
		/**Temperature value, bits 8-15*/
		uint8_t TemperatureHigh;
	};
} tSTC31000Data;

typedef struct {
	/**Value of the voltage in mV.*/
	float Voltage;
	/**Value of the current in mA*/
	float Current;
	/**Value of the temperature in C*/
	float Temperature;
	/**Value of the current charge of the battery in mAh*/
	float Charge;
} tBatteryData;

/*=========================================================================*/

class STC3100 : public I2C_Device {
public:
	STC3100(int32_t sensorID = -1);

	bool init(uint32_t r_sens = 20,
			stc3100_res_t res = STC3100_MODE_ULTRAHIGHRES);

	bool refresh();

	float getCurrent() { return _current;}
	float getVoltage() {return _voltage;}
	float getTemperature() {return _temperature;}
	float getCorrectedVoltage(float int_res);

	float getCharge() const {
		return _charge;
	}

private:
	int32_t _sensorID;
	uint8_t _deviceID, _stc3100Mode;
	int32_t _r_sens;
	tSTC31000Data _stc_data;

	// voltage
	float _voltage;
	/**Value of the current in mA*/
	float _current;
	/**Value of the temperature in C*/
	float _temperature;
	/**Value of the current charge of the battery in mAh*/
	float _charge;
	// counter
	float _counter;

	void writeCommand(uint8_t reg, uint8_t value);
	void readChip();
	void computeVoltage();
	void computeCharge();
	void computeCurrent();
	void computeCounter();
	void computeTemp();

};

#endif /* LIBRARIES_STC3100_H_ */
