/*
 * MMA8451.h
 *
 *  Created on: 26 févr. 2017
 *      Author: Vincent
 */

#ifndef LIBRARIES_MMA8451_H_
#define LIBRARIES_MMA8451_H_

#include "Arduino.h"
#include "Serial.h"
#include "I2CDEVICE.h"

#define INT_PIN1    7
#define INT_PIN2    8

#define SENSORS_GRAVITY_EARTH             (9.80665F)              /**< Earth's gravity in m/s^2 */
#define SENSORS_GRAVITY_MOON              (1.6F)                  /**< The moon's gravity in m/s^2 */
#define SENSORS_GRAVITY_SUN               (275.0F)                /**< The sun's gravity in m/s^2 */
#define SENSORS_GRAVITY_STANDARD          (SENSORS_GRAVITY_EARTH)
#define SENSORS_MAGFIELD_EARTH_MAX        (60.0F)                 /**< Maximum magnetic field on Earth's surface */
#define SENSORS_MAGFIELD_EARTH_MIN        (30.0F)                 /**< Minimum magnetic field on Earth's surface */
#define SENSORS_PRESSURE_SEALEVELHPA      (1013.25F)              /**< Average sea level pressure is 1013.25 hPa */
#define SENSORS_DPS_TO_RADS               (0.017453293F)          /**< Degrees/s to rad/s multiplier */
#define SENSORS_GAUSS_TO_MICROTESLA       (100)                   /**< Gauss to micro-Tesla multiplier */


const byte REG_STATUS = 0x00; //(R) Real time status
const byte REG_OUT_X_MSB = 0x01; //(R) [7:0] are 8 MSBs of 14-bit sample
const byte REG_OUT_X_LSB = 0x02; //(R) [7:2] are 2 LSBs of 14-bit sample
const byte REG_OUT_Y_MSB = 0x03; //(R) [7:0] are 8 MSBs of 14-bit sample
const byte REG_OUT_Y_LSB = 0x04; //(R) [7:2] are 2 LSBs of 10-bit sample
const byte REG_OUT_Z_MSB = 0x05; //(R) [7:0] are 8 MSBs of 10-bit sample
const byte REG_OUT_Z_LSB = 0x06; //(R) [7:2] are 2 LSBs of 10-bit sample
const byte REG_F_SETUP = 0x09; //FIFO Setup
const byte REG_TRIG_CFG = 0x0a;
const byte REG_SYSMOD = 0x0b; //(R) Current system mode
const byte REG_INT_SOURCE = 0x0c; //(R) Interrupt status
const byte REG_WHO_AM_I = 0x0d; //(R) Device ID (0x1A)
const byte REG_XYZ_DATA_CFG = 0xe; //(R/W) Dynamic range settings
const byte REG_HP_FILTER_CUTOFF = 0x0f; //(R/W) cut-off frequency is set to 16Hz @ 800Hz
const byte REG_PL_STATUS = 0x10; //(R) Landscape/Portrait orientation status
const byte REG_PL_CFG = 0x11; //(R/W) Landscape/Portrait configuration
const byte REG_PL_COUNT = 0x12; //(R) Landscape/Portrait debounce counter
const byte REG_PL_BF_ZCOMP = 0x13; //(R) Back-Front, Z-Lock trip threshold
const byte REG_P_L_THS_REG = 0x14; //(R/W) Portrait to Landscape trip angle is 29 degree
const byte REG_FF_MT_CFG = 0x15; //(R/W) Freefall/motion functional block configuration
const byte REG_FF_MT_SRC = 0x16; //(R) Freefall/motion event source register
const byte REG_FF_MT_THS = 0x17; //(R/W) Freefall/motion threshold register
const byte REG_FF_MT_COUNT = 0x18; //(R/W) Freefall/motion debounce counter
const byte REG_TRANSIENT_CFG = 0x1d; //(R/W) Transient functional block configuration
const byte REG_TRANSIENT_SRC = 0x1e; //(R) Transient event status register
const byte REG_TRANSIENT_THS = 0x1f; //(R/W) Transient event threshold
const byte REG_TRANSIENT_COUNT = 0x20; //(R/W) Transient debounce counter
const byte REG_PULSE_CFG = 0x21; //(R/W) ELE, Double_XYZ or Single_XYZ
const byte REG_PULSE_SRC = 0x22; //(R) EA, Double_XYZ or Single_XYZ
const byte REG_PULSE_THSX = 0x23; //(R/W) X pulse threshold
const byte REG_PULSE_THSY = 0x24; //(R/W) Y pulse threshold
const byte REG_PULSE_THSZ = 0x25; //(R/W) Z pulse threshold
const byte REG_PULSE_TMLT = 0x26; //(R/W) Time limit for pulse
const byte REG_PULSE_LTCY = 0x27; //(R/W) Latency time for 2nd pulse
const byte REG_PULSE_WIND = 0x28; //(R/W) Window time for 2nd pulse
const byte REG_ASLP_COUNT = 0x29; //(R/W) Counter setting for auto-sleep
const byte REG_CTRL_REG1 = 0x2a; //(R/W) ODR = 800 Hz, STANDBY mode
const byte REG_CTRL_REG2 = 0x2b; //(R/W) Sleep enable, OS Modes, RST, ST
const byte REG_CTRL_REG3 = 0x2c; //(R/W) Wake from sleep, IPOL, PP_OD
const byte REG_CTRL_REG4 = 0x2d; //(R/W) Interrupt enable register
const byte REG_CTRL_REG5 = 0x2e; //(R/W) Interrupt pin (INT1/INT2) map
const byte REG_OFF_X = 0x2f; //(R/W) X-axis offset adjust
const byte REG_OFF_Y = 0x30; //(R/W) Y-axis offset adjust
const byte REG_OFF_Z = 0x31; //(R/W) Z-axis offset adjust

const byte FULL_SCALE_RANGE_2g = 0x0;
const byte FULL_SCALE_RANGE_4g = 0x1;
const byte FULL_SCALE_RANGE_8g = 0x2;

const byte odrMask = 0x38;
const byte ODR_800Hz = 0x00;
const byte ODR_400Hz = 0x01;
const byte ODR_200Hz = 0x02;
const byte ODR_100Hz = 0x03;
const byte ODR_50Hz = 0x04;
const byte ODR_12_5Hz = 0x05;
const byte ODR_6_25Hz = 0x06;
const byte ODR_1_563Hz = 0x07;

const byte modsMask = 0x03;
const byte MODS_Normal = 0x00;
const byte MODS_LowNoiseLowPower = 0x01;
const byte MODS_HighResolution = 0x02;
const byte MODS_LowPower = 0x03;

const byte HPFCutOffFreqMask = 0x03;
const byte HPFMask = 0x10;

const byte activeMask = 0x01;
const byte resModeMask = 0x02;
const byte lowNoiseMask = 0x04;

extern "C" void accelPulseISR(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t polarity_);
extern "C" void accelShakeISR(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t polarity_);

extern Serial serial;

class MMA8451_n0m1 : public I2C_Device {

public:
	friend void accelISR(void); //make friend so bttnPressISR can access private var keyhit

	MMA8451_n0m1();

	virtual bool init();
	bool init_bare();

	void setI2CAddr(int address);

	/***********************************************************
	 *
	 * dataMode
	 *
	 * set the device to return raw data values
	 *
	 ***********************************************************/
	void dataMode(boolean highRes, int gScaleRange);

	/***********************************************************
	 *
	 * dataModeInt
	 *
	 * set the device to return raw data values
	 *
	 ***********************************************************/
	void dataModeInt(boolean highRes, int gScaleRange, byte odr,
	boolean enableINT2, int arduinoINTPin);

	int x() {
		return x_;
	}

	int y() {
		return y_;
	}

	int z() {
		return z_;
	}

	float xg() {
		return x_g;
	}

	float yg() {
		return y_g;
	}

	float zg() {
		return z_g;
	}

	unsigned long measure_time() {
		return measure_time_;
	}

	void xyzt(int& x, int& y, int& z, unsigned long& t) {

		if (shakeMode_ == true || motionMode_ == true || dataModeInt_ == true) {
			detachISRProc();
		}

		x = x_;
		y = y_;
		z = z_;
		t = measure_time_;

		if (shakeMode_ == true || motionMode_ == true || dataModeInt_ == true) {
			attachISRProc();
		}
	}

	/***********************************************************
	 *
	 * shakeMode / tapMode
	 *
	 *  set to transient detection mode
	 *
	 ***********************************************************/

	void pulseMode(boolean enableX, boolean enableY,
			boolean enableZ, boolean enableINT2);

	void shakeMode(int threshold, boolean enableX, boolean enableY,
			boolean enableZ, boolean enableINT2);

	boolean shake() {
		boolean shakeOut = shake_;
		shake_ = false;
		return shakeOut;
	}
	boolean shakeAxisX() {
		boolean shakeAxisOut = shakeAxisX_;
		shakeAxisX_ = false;
		return shakeAxisOut;
	}
	boolean shakeAxisY() {
		boolean shakeAxisOut = shakeAxisY_;
		shakeAxisY_ = false;
		return shakeAxisOut;
	}
	boolean shakeAxisZ() {
		boolean shakeAxisOut = shakeAxisZ_;
		shakeAxisZ_ = false;
		return shakeAxisOut;
	}

	boolean pulse() {
		boolean pulseOut = pulse_;
		pulse_ = false;
		return pulseOut;
	}
	boolean pulseX() {
		uint8_t pulseAxisOut = pulseAxisX_;
		pulseAxisX_ = 0;
		return pulseAxisOut;
	}
	boolean pulseY() {
		uint8_t pulseAxisOut = pulseAxisY_;
		pulseAxisY_ = 0;
		return pulseAxisOut;
	}
	boolean pulseZ() {
		uint8_t pulseAxisOut = pulseAxisZ_;
		pulseAxisZ_ = 0;
		return pulseAxisOut;
	}

	void motionMode(int threshold, boolean enableX, boolean enableY,
			boolean enableZ, boolean enableINT2, int arduinoINTPin);

	boolean motion() {
		boolean motionOut = motion_;
		motion_ = false;
		return motionOut;
	}

	boolean dataready() {
		boolean datareadyOut = dataready_;
		dataready_ = false;
		return datareadyOut;
	}

	/***********************************************************
	 *
	 * update
	 *
	 * update data values, or clear interrupts. Use at start of loop()
	 *
	 ***********************************************************/
	virtual void update();

	/***********************************************************
	 *
	 * reset
	 *
	 * update data values, or clear interrupts. Use at start of loop()
	 *
	 ***********************************************************/
	void reset();

	void regRead(byte reg, byte *buf, byte count = 1);

	void regWrite(byte reg, byte val);

	void setODR(byte odr = ODR_12_5Hz);

	void setOversampling(byte mods = MODS_Normal);

	void setHPF(boolean filter = false);

	void setHPF_Cutoff_freq(byte freq = 0);

	void setActive();

	boolean setStandby();

	void attachISRProc();

	void detachISRProc();

	void setDebug() {
		debug = true;
	}
	void clearDebug() {
		debug = false;
	}

	void setLNoise();
	void clearLNoise();

	void setOFFX(byte off_x);
	void setOFFY(byte off_y);
	void setOFFZ(byte off_z);

	char getOFFX();
	char getOFFY();
	char getOFFZ();

//-----------------------------------------------------------
// Compatiblity functions to match the api of the ADXL345 library
// http://code.google.com/p/adxl345driver/
// allows for more easy updates of code from the ADXL345 to the MMA8451
//-----------------------------------------------------------
	void setRangeSetting(int gScaleRange) {
		gScaleRange_ = gScaleRange;
	} //call this before setFullResBit()
	void setFullResBit(boolean highRes) {
		dataMode(highRes, gScaleRange_);
	}
	void readAccel(int *x, int *y, int *z) {
		xyz(*x, *y, *z);
	}

	volatile boolean shakeISRFlag;
	volatile boolean pulseISRFlag;
	volatile unsigned long measure_time_;
	static MMA8451_n0m1* pMMA8451_n0m1; //ptr to MMA8451_n0m1 class for the ISR

private:

	void xyz(int& x, int& y, int& z);
	void clearInterrupt();

	virtual bool wireWriteDataByte(uint8_t reg, uint8_t val);
	virtual bool wireReadDataByte(uint8_t reg, uint8_t &val);
	virtual int wireReadDataBlock(uint8_t reg, uint8_t *val, unsigned int len);

	int x_, y_, z_;
	byte off_x, off_y, off_z;
	float x_g, y_g, z_g;

	byte I2CAddr;
	boolean highRes_;
	int gScaleRange_;
	boolean dataMode_;
	boolean dataModeInt_;
	boolean shakeMode_;
	boolean motionMode_;
	boolean tapMode_;

	boolean motion_;
	boolean shake_;
	boolean pulse_;

	boolean dataready_;
	boolean shakeAxisX_;
	boolean shakeAxisY_;
	boolean shakeAxisZ_;

	int8_t pulseAxisX_;
	int8_t pulseAxisY_;
	int8_t pulseAxisZ_;

	boolean debug;

};

#endif /* LIBRARIES_MMA8451_H_ */
