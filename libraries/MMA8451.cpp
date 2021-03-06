/*
 * MMA8451.cpp
 *
 *  Created on: 26 f�vr. 2017
 *      Author: Vincent
 */

#include "I2C.h"
#include "Arduino.h"
#include "MMA8451.h"

#define NRF_LOG_MODULE_NAME "MMA"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include "Serial.h"
extern Serial serial;

// 7 bits I2C address
#define MMA8451_ADDRESS  0x1D

MMA8451_n0m1* MMA8451_n0m1::pMMA8451_n0m1 = 0;


/***********************************************************
 *
 * accelPulseISR
 *
 *
 *
 ***********************************************************/
void int2ISR(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t polarity_) {
	if (polarity_ == NRF_GPIOTE_POLARITY_HITOLO) {
		MMA8451_n0m1::pMMA8451_n0m1->int2ISRFlag = true;
	}
}

/***********************************************************
 *
 * accelShakeISR
 *
 *
 *
 ***********************************************************/
void int1ISR(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t polarity_) {
	if (polarity_ == NRF_GPIOTE_POLARITY_HITOLO) {
		MMA8451_n0m1::pMMA8451_n0m1->int1ISRFlag = true;
	}
}

MMA8451_n0m1::MMA8451_n0m1() : I2C_Device () {
	pMMA8451_n0m1 = this;
	dataMode_ = false;
	shakeMode_ = false;
	int1ISRFlag = false;
	int2ISRFlag = false;
	shake_ = false;
	shakeAxisX_ = false;
	shakeAxisY_ = false;
	shakeAxisZ_ = false;
	pulseAxisX_ = 0;
	pulseAxisY_ = 0;
	pulseAxisZ_ = 0;
	I2CAddr = MMA8451_ADDRESS;
	gScaleRange_ = 2;  //default 2g
	off_x = 0;
	off_y = 0;
	off_z = 0;

	debug = false;
}

/***********************************************************
 *
 * setI2CAddr
 *
 *
 *
 ***********************************************************/
void MMA8451_n0m1::setI2CAddr(int address) {
	I2CAddr = address;
}

// init
bool MMA8451_n0m1::init() {

	uint8_t statusCheck;

	if (!this->checkIfPresent(MMA8451_ADDRESS)) {
		return false;
	}

	wireReadDataByte(REG_WHO_AM_I, statusCheck);
	NRF_LOG_INFO("Device ID: 0x%x\r\n", statusCheck);
	NRF_LOG_FLUSH();

	if (statusCheck != 0x1A && statusCheck != 0xC7) {
		NRF_LOG_ERROR("Wrong device ID !!\r\n");
		return false;
	}

	// TODO reset
	reset();

	setStandby();

	init_bare();

	// go !!
	setActive();

	// attach interrupts
	attachInterrupt(INT_PIN1, int1ISR);
	attachInterrupt(INT_PIN2, int2ISR);


	return true;
}


// init
bool MMA8451_n0m1::init_bare() {

	// setup datarate
	setODR(ODR_400Hz); //Set device in ODR rate

	// setup high-level functions
	// acc values INT2
	dataModeInt(true, 8, true);
	// INT1 X-Z
	pulseMode(true,false,true,false);
	// INT1 Y
	shakeMode(10,false,true,false,false);

#if 0
	uint8_t statusCheck;
	// setup auto sleep
	// enable auto sleep, set sleep power mode scheme
	wireReadDataByte(REG_CTRL_REG2, statusCheck);
	statusCheck &= ~(0b00011100);
	statusCheck |=   0b00011100; // select low power mode
	wireWriteDataByte(REG_CTRL_REG2, statusCheck);

	// sleep datarate
	wireReadDataByte(REG_CTRL_REG1, statusCheck);
	statusCheck &= ~(0b11000000);
	statusCheck |=   0b11000000; // select 50 Hz sleep mode sample frequency
	wireWriteDataByte(REG_CTRL_REG1, statusCheck);

	// setup auto-sleep timeout
	wireWriteDataByte(REG_ASLP_COUNT, 0x30);

	// sleep interrupt enable
	wireReadDataByte(REG_CTRL_REG4, statusCheck);
	statusCheck &= ~(0b10000000); // clear bit 7
	statusCheck |=   0b10110000; // select  Auto-SLEEP/WAKE interrupt enable, transient interrupt, orientation
	wireWriteDataByte(REG_CTRL_REG4, statusCheck);

	// sleep interrupt enable
	wireReadDataByte(REG_CTRL_REG5, statusCheck);
	statusCheck &= ~(0b10000000); // clear bit 7
	statusCheck |=   0b00000000; // route sleep INT to pin 2
	wireWriteDataByte(REG_CTRL_REG4, statusCheck);

	// sleep interrupt scheme
	wireReadDataByte(REG_CTRL_REG3, statusCheck);
	statusCheck &= ~(0b01111111);
	statusCheck |= 0b01010101; // wakeup by tap/shake/vectm + active low/open-drain
	wireWriteDataByte(REG_CTRL_REG3, statusCheck);

#endif

	return true;
}


/***********************************************************
 *
 * update
 *
 *
 *
 ***********************************************************/
void MMA8451_n0m1::update() {

	if (dataMode_) {
		if (debug)
			serial.println("in dataMode_");
		xyz(x_, y_, z_);
	}

	this->run();

}

void MMA8451_n0m1::run() {

	if (tapMode_ == true || shakeMode_ == true || motionMode_ == true || dataModeInt_ == true) {
		clearInterrupt();
	}

}

/***********************************************************
 *
 * clearInterrupt
 *
 *
 *
 ***********************************************************/
void MMA8451_n0m1::clearInterrupt() {

	if (debug)
		serial.print("(clearInterrupt) ");

	// TODO check that
	if (int2ISRFlag || int1ISRFlag ||
			digitalRead(INT_PIN1)==LOW || digitalRead(INT_PIN2)==LOW) {

		int2ISRFlag = int1ISRFlag = false;

		if (debug)
			serial.println(" ISRFlag ON");

		byte sourceSystem;
		wireReadDataByte(REG_INT_SOURCE, sourceSystem);

		if ((sourceSystem & 0x80) == 0x80) {
		} //Auto-sleep/Wake

		if ((sourceSystem & 0x40) == 0x40) {
		} //FIFO

		if ((sourceSystem & 0x20) == 0x20) { //Transient
			//Perform an Action since Transient Flag has been set
			//Read the Transient to clear system interrupt and Transient
			byte srcTrans;
			shake_ = true;
			wireReadDataByte(REG_TRANSIENT_SRC, srcTrans);

			if ((srcTrans & 0x02) == 0x02) {
				serial.println("Shake on X"); // tabbing here for visibility
				// MMA is 90 degres
				// TODO this->notify(SHAKE, Xp);
				shakeAxisX_ = true;
			}
			if ((srcTrans & 0x08) == 0x08) {
				serial.println("Shake on Y"); // tabbing here for visibility
				this->notify(SHAKE, Xp);
				shakeAxisY_ = true;
			}
			if ((srcTrans & 0x20) == 0x20) {
				serial.println("Shake on Z"); // tabbing here for visibility
				// TODO this->notify(SHAKE, Zp);
				shakeAxisZ_ = true;
			}
		}

		if ((sourceSystem & 0x10) == 0x10) {
		} //Landscape/Portrait

		if ((sourceSystem & 0x08) == 0x08) {
			//Perform an Action since Pulse Flag has been set
			// Read the Pulse to clear system interrupt and Transient
			byte source;
			shake_ = true;

			// read register
			wireReadDataByte(REG_PULSE_SRC, source);

			if ((source & 0x10) == 0x10) { // If AxX bit is set

				if ((source & 0x08) == 0x08) { // If DPE (double puls) bit is set
					serial.print("    Double Tap (2) on X"); // tabbing here for visibility
					pulseAxisX_ = 2;
				} else {
					serial.print("Single (1) tap on X");
					pulseAxisX_ = 1;
				}

				if ((source & 0x01) == 0x01) { // If PoIX is set
					serial.println(" -");
					this->notify(pulseAxisX_==1?STAP:DTAP, Xm);
					pulseAxisX_ = -pulseAxisX_;
				} else {
					serial.println(" +");
					this->notify(pulseAxisX_==1?STAP:DTAP, Xp);
				}
			}
			if ((source & 0x20) == 0x20) { // If AxY bit is set

				if ((source & 0x08) == 0x08) { // If DPE (double pulse) bit is set
					serial.print("    Double Tap (2) on Y");
					pulseAxisY_ = 2;
				} else {
					serial.print("Single (1) tap on Y");
					pulseAxisY_ = 1;
				}

				if ((source & 0x02) == 0x02) { // If PoIY is set
					serial.println(" -");
					this->notify(pulseAxisY_==1?STAP:DTAP, Ym);
					pulseAxisY_ = -pulseAxisY_;
				} else {
					serial.println(" +");
					this->notify(pulseAxisY_==1?STAP:DTAP, Yp);
				}
			}
			if ((source & 0x40) == 0x40) { // If AxZ bit is set

				if ((source & 0x08) == 0x08) { // If DPE (double pulse) bit is set
					serial.print("    Double Tap (2) on Z");
					pulseAxisZ_ = 2;
				} else {
					serial.print("Single (1) tap on Z");
					pulseAxisZ_ = 1;
				}


			    if ((source & 0x04)==0x04) { // If PoIZ is set
			      serial.println(" -");
			      this->notify(pulseAxisZ_==1?STAP:DTAP, Zm);
			      pulseAxisZ_ = -pulseAxisZ_;
			    } else {
			      serial.println(" +");
			      this->notify(pulseAxisZ_==1?STAP:DTAP, Zp);
			    }
			}

		} //fin Pulse

		if ((sourceSystem & 0x04) == 0x04) //FreeFall Motion
				{
			byte srcFF;
			wireReadDataByte(REG_FF_MT_SRC, srcFF);
			motion_ = true;
//		xyz(x_,y_,z_);

		}

		if ((sourceSystem & 0x01) == 0x01) { //DataReady
			if (debug)
				serial.println("Processing Dataready interrupt");
//			detachISRProc();
			xyz(x_, y_, z_);
//			attachISRProc();
			dataready_ = true;
		}

	} else {
		if (debug) {
			serial.println(" ISRFlag OFF dumping accelrator registers ");

			byte buf[0x32];
			wireReadDataBlock(REG_OUT_X_MSB, buf, 0x32);

		}
	}

}


/*************************************************************
 *
 * xyz
 *
 * Get accelerometer readings (x, y, z)
 * by default, standard 14 bits mode is used.
 *
 * This function also convers 2's complement number to
 * signed integer result.
 *
 *************************************************************/
void MMA8451_n0m1::xyz(int& x, int& y, int& z) {

	if (debug)
		serial.println("xyz()");

	byte buf[6];

	if (highRes_) {
		wireReadDataBlock(REG_OUT_X_MSB, buf, 6);

		x = (int16_t)(((buf[0] << 8) | buf[1])) >> 2;
		y = (int16_t)(((buf[2] << 8) | buf[3])) >> 2;
		z = (int16_t)(((buf[4] << 8) | buf[5])) >> 2;

	} else {
		wireReadDataBlock(REG_OUT_X_MSB, buf, 3);

		x = (int16_t)(buf[0] << 6);
		y = (int16_t)(buf[1] << 6);
		z = (int16_t)(buf[2] << 6);
	}

	uint16_t divider = 1;
	switch (gScaleRange_) {
	case FULL_SCALE_RANGE_2g:
		divider = 4096;
		break;
	case FULL_SCALE_RANGE_4g:
		divider = 2048;
		break;
	case FULL_SCALE_RANGE_8g:
		divider = 1024;
		break;
	}

	x_g = (float)x * SENSORS_GRAVITY_STANDARD / divider;

	y_g = (float)y * SENSORS_GRAVITY_STANDARD / divider;

	z_g = (float)z * SENSORS_GRAVITY_STANDARD / divider;

}

/***********************************************************
 *
 * dataMode
 *
 *
 *
 ***********************************************************/
void MMA8451_n0m1::dataMode(boolean highRes, int gScaleRange) {
	highRes_ = highRes;
	gScaleRange_ = gScaleRange;
	dataMode_ = true;
	byte statusCheck;

	if (gScaleRange_ <= 3) {
		gScaleRange_ = FULL_SCALE_RANGE_2g;
	} //0-3 = 2g
	else if (gScaleRange_ <= 5) {
		gScaleRange_ = FULL_SCALE_RANGE_4g;
	} //4-5 = 4g
	else if (gScaleRange_ <= 8) {
		gScaleRange_ = FULL_SCALE_RANGE_8g;
	} // 6-8 = 8g
	else if (gScaleRange_ > 8) {
		gScaleRange_ = FULL_SCALE_RANGE_8g;
	} //boundary
	wireWriteDataByte(REG_XYZ_DATA_CFG, gScaleRange_);

	//set highres 14bit or lowres 8bit
	wireReadDataByte(REG_CTRL_REG1, statusCheck);
	if (highRes) {
		wireWriteDataByte(REG_CTRL_REG1, (statusCheck & ~resModeMask));
	} else {
		wireWriteDataByte(REG_CTRL_REG1, (statusCheck | resModeMask));
	}

}

/***********************************************************
 *
 * dataModeInt
 *
 *
 *
 ***********************************************************/

void MMA8451_n0m1::dataModeInt(boolean highRes, int gScaleRange, boolean enableINT2) {
	highRes_ = highRes;
	gScaleRange_ = gScaleRange;
	byte statusCheck;

	if (gScaleRange_ <= 3) {
		gScaleRange_ = FULL_SCALE_RANGE_2g;
	} //0-3 = 2g
	else if (gScaleRange_ <= 5) {
		gScaleRange_ = FULL_SCALE_RANGE_4g;
	} //4-5 = 4g
	else if (gScaleRange_ <= 8) {
		gScaleRange_ = FULL_SCALE_RANGE_8g;
	} // 6-8 = 8g
	else if (gScaleRange_ > 8) {
		gScaleRange_ = FULL_SCALE_RANGE_8g;
	} //boundary
	wireWriteDataByte(REG_XYZ_DATA_CFG, gScaleRange_);

	//set highres 14bit or lowres 8bit
	wireReadDataByte(REG_CTRL_REG1, statusCheck);
	if (highRes) {
		wireWriteDataByte(REG_CTRL_REG1, (statusCheck & ~resModeMask));
	} else {
		wireWriteDataByte(REG_CTRL_REG1, (statusCheck | resModeMask));
	}

	// accelerator vector magnitude interrupt enable
	wireReadDataByte(REG_CTRL_REG4, statusCheck);
	statusCheck |= 0x10;
	wireWriteDataByte(REG_CTRL_REG4, statusCheck);

	// route interrupt
	wireReadDataByte(REG_CTRL_REG5, statusCheck);
	if (!enableINT2) {
		statusCheck |= 0x10;
	} else {
		statusCheck &= ~0x10;
	}
	wireWriteDataByte(REG_CTRL_REG5, statusCheck);

}

/***********************************************************
 *
 * motionMode
 *
 *
 *
 ***********************************************************/
void MMA8451_n0m1::motionMode(int threshold, boolean enableX, boolean enableY,
boolean enableZ, boolean enableINT2, int arduinoINTPin) {

	boolean error = false;
	byte statusCheck;

	//wireWriteDataByte(REG_CTRL_REG1, 0x18); //Set device in 100 Hz ODR, Standby

	byte xyzCfg = 0x80; //latch always enabled
	xyzCfg |= 0x40; //Motion not free fall
	if (enableX)
		xyzCfg |= 0x08;
	if (enableY)
		xyzCfg |= 0x10;
	if (enableZ)
		xyzCfg |= 0x20;

	wireWriteDataByte(REG_FF_MT_CFG, xyzCfg); //XYZ + latch + motion
	wireReadDataByte(REG_FF_MT_CFG, statusCheck);
	if (statusCheck != xyzCfg)
		error = true;

	if (threshold > 127)
		threshold = 127; //a range of 0-127.
	wireWriteDataByte(REG_FF_MT_THS, threshold); //threshold
	wireReadDataByte(REG_FF_MT_THS, statusCheck);
	if (statusCheck != byte(threshold))
		error = true;

	wireWriteDataByte(REG_FF_MT_COUNT, 0x0A); //Set the Debounce Counter for 100 ms
	wireReadDataByte(REG_FF_MT_COUNT, statusCheck);
	if (statusCheck != 0x0A)
		error = true;

	wireReadDataByte(REG_CTRL_REG4, statusCheck);
	statusCheck |= 0b100;
	wireWriteDataByte(REG_CTRL_REG4, statusCheck); //Enable Motion Interrupt in the System

	// TODO ISR
//	byte intSelect = 0x04;
//	if (enableINT2)
//		intSelect = 0x00;
//	wireReadDataByte(REG_CTRL_REG5, statusCheck);
//	statusCheck |= intSelect;
//	wireWriteDataByte(REG_CTRL_REG5, statusCheck); //INT2 0x0, INT1 0x04

//	wireReadDataByte(REG_CTRL_REG1, statusCheck); //Read out the contents of the register
//	statusCheck |= 0x01; //Change the value in the register to Active Mode.
//	wireWriteDataByte(REG_CTRL_REG1, statusCheck);

	if (error) {
		serial.println("Motion mode setup error");
		serial.println("retrying...");
		delay(100);
		motionMode(threshold, enableX, enableY, enableZ, enableINT2,
				arduinoINTPin);
	}

	motionMode_ = true;
}


/***********************************************************
 *
 * shakeMode
 *
 *
 *
 ***********************************************************/
void MMA8451_n0m1::shakeMode(int threshold, boolean enableX, boolean enableY,
boolean enableZ, boolean enableINT2) {

	boolean error = false;
	byte statusCheck;

	byte xyzCfg = 0x10; //latch always enabled
	if (enableX)
		xyzCfg |= 0x02;
	if (enableY)
		xyzCfg |= 0x04;
	if (enableZ)
		xyzCfg |= 0x08;

	wireWriteDataByte(REG_TRANSIENT_CFG, xyzCfg); //XYZ + latch 0x1E
	wireReadDataByte(REG_TRANSIENT_CFG, statusCheck);
	if (statusCheck != xyzCfg)
		error = true;

	if (threshold > 127)
		threshold = 127; //8g is the max.
	wireWriteDataByte(REG_TRANSIENT_THS, threshold);
	wireReadDataByte(REG_TRANSIENT_THS, statusCheck);
	if (statusCheck != (uint8_t)(threshold))
		error = true;

	//Set the Debounce Counter for 150 ms
	wireWriteDataByte(REG_TRANSIENT_COUNT, 0x3C);
	wireReadDataByte(REG_TRANSIENT_COUNT, statusCheck);
	if (statusCheck != 0x3C)
		error = true;

	//Enable Transient Detection Interrupt in the System
	wireReadDataByte(REG_CTRL_REG4, statusCheck);
	statusCheck |= 0b100000;
	wireWriteDataByte(REG_CTRL_REG4, statusCheck);

	byte intSelect;
	if (enableINT2) {
		// INT2
		intSelect = 0x00;
	} else {
		// INT1
		intSelect = 0x20;
	}
	wireReadDataByte(REG_CTRL_REG5, statusCheck);
	statusCheck |= intSelect;
	wireWriteDataByte(REG_CTRL_REG5, statusCheck); //INT2 0x0, INT1 0x20

	if (error) {
		serial.println("Shake mode setup error");
		serial.println("retrying...");
		delay(100);
		shakeMode(threshold, enableX, enableY, enableZ, enableINT2);
	}

	shakeMode_ = true;

}


/* Set up single and double tap - 5 steps:
   1. Set up single and/or double tap detection on each axis individually.
   2. Set the threshold - minimum required acceleration to cause a tap.
   3. Set the time limit - the maximum time that a tap can be above the threshold
   4. Set the pulse latency - the minimum required time between one pulse and the next
   5. Set the second pulse window - maximum allowed time between end of latency and start of second pulse
   for more info check out this app note: http://cache.freescale.com/files/sensors/doc/app_note/AN4072.pdf */
void MMA8451_n0m1::pulseMode(boolean enableX, boolean enableY, boolean enableZ, boolean enableINT2) {

	boolean error = false;
	byte statusCheck;

	// HP enabled + LP enabled
	wireWriteDataByte(REG_HP_FILTER_CUTOFF, 0b10000);

	// TODO
	//wireWriteDataByte(REG_PULSE_CFG, 0x3F);  // 1. enable single/double taps on all axes
	//wireWriteDataByte(REG_PULSE_CFG, 0x15);  // 1. single taps only on all axes
	wireWriteDataByte(REG_PULSE_CFG, 0x6A);  // 1. double taps only on all axes

	// 2. x thresh at 4g, multiply the value by 0.0625g/LSB to get the threshold
	wireWriteDataByte(REG_PULSE_THSX, 0x4A);

	// 2. y thresh at 4g, multiply the value by 0.0625g/LSB to get the threshold
	wireWriteDataByte(REG_PULSE_THSY, 0x4A);

	// 2. z thresh at 5g, multiply the value by 0.0625g/LSB to get the threshold
	wireWriteDataByte(REG_PULSE_THSZ, 0x4A);

	// 3. 25ms time limit at 100Hz odr, this is very dependent on data rate, see the app note
	// = 0.625ms(ODR=100,HighRes) * TMLT
	wireWriteDataByte(REG_PULSE_TMLT, 0x0A);

	// 4. 120ms between taps min, this also depends on the data rate
	// = 1.25ms(ODR=100,HighRes) * LTCY
	wireWriteDataByte(REG_PULSE_LTCY, 0x14);

	// 5. 318ms at 100 Hz between taps max
	// = 1.25ms(ODR=100,HighRes) * WIND
	wireWriteDataByte(REG_PULSE_WIND, 0x5A);

	// tap ints enabled
	wireReadDataByte(REG_CTRL_REG4, statusCheck);
	statusCheck |= 0b1000;
	wireWriteDataByte(REG_CTRL_REG4, statusCheck);

	// configure interrupt
	byte intSelect;
	if (enableINT2) {
		// INT2
		intSelect = 0x0000;
	} else {
		// INT1
		intSelect = 0b1000;
	}
	wireReadDataByte(REG_CTRL_REG5, statusCheck);
	statusCheck |= intSelect;
	wireWriteDataByte(REG_CTRL_REG5, statusCheck); //INT2 0x0, INT1 0x20

	if (error) {
		serial.println("Tap mode setup error");
		serial.println("retrying...");
		delay(100);
		pulseMode(enableX, enableY, enableZ, enableINT2);
	}

	tapMode_ = true;
}

/***********************************************************
 *
 * regRead
 *
 *
 *
 ***********************************************************/
void MMA8451_n0m1::regRead(byte reg, byte *buf, byte count) {
	wireReadDataBlock(reg, buf, count);
}

/***********************************************************
 *
 * regWrite
 *
 *
 *
 ***********************************************************/
void MMA8451_n0m1::regWrite(byte reg, byte val) {
	wireWriteDataByte(reg, val);
}



/***********************************************************
 *
 * setODR
 *
 *
 *
 ***********************************************************/
void MMA8451_n0m1::setODR(byte odr) {
	byte statusCheck;

	odr <<= 3;

	//register settings must be made in standby mode
	boolean was_active = setStandby();

	wireReadDataByte(REG_CTRL_REG1, statusCheck);
	statusCheck = statusCheck & ~odrMask;
	wireWriteDataByte(REG_CTRL_REG1, (statusCheck | odr));

	//active Mode if was active
	if (was_active)
		setActive();
}

/***********************************************************
 *
 * reset
 *
 *
 *
 ***********************************************************/
void MMA8451_n0m1::reset() {

	uint32_t millis_ = millis();
	//Reset device
	wireWriteDataByte(REG_CTRL_REG2, 0x40);
	delay(2);
	uint8_t val = 0x40;
// TODO check
	while (val & 0x40 && (millis() - millis_ < 1000)) {
		wireReadDataByte(REG_CTRL_REG2, val);
	}
	NRF_LOG_INFO("Reset took %u us\r\n", millis() - millis_);

}

/***********************************************************
 *
 * setOversampling
 *
 *
 *
 ***********************************************************/
void MMA8451_n0m1::setOversampling(byte mods) {
	byte statusCheck;

	//register settings must be made in standby mode
	boolean was_active = setStandby();

	wireReadDataByte(REG_CTRL_REG2, statusCheck);
	statusCheck = statusCheck & ~modsMask;
	wireWriteDataByte(REG_CTRL_REG2, (statusCheck | mods));

	//active Mode if was active
	if (was_active)
		setActive();
}

/***********************************************************
 *
 * setHPF_Cutoff_freq
 *
 *
 *
 ***********************************************************/
void MMA8451_n0m1::setHPF_Cutoff_freq(byte freq) {
	byte statusCheck;

	freq = freq & HPFCutOffFreqMask;

	//register settings must be made in standby mode
	boolean was_active = setStandby();

	wireReadDataByte(REG_HP_FILTER_CUTOFF, statusCheck);
	statusCheck = statusCheck & ~HPFCutOffFreqMask;
	wireWriteDataByte(REG_HP_FILTER_CUTOFF, (statusCheck | freq));

	//active Mode if was active
	if (was_active)
		setActive();
}

/***********************************************************
 *
 * setHPF
 *
 *
 *
 ***********************************************************/
void MMA8451_n0m1::setHPF(boolean filter) {
	byte statusCheck;

	//register settings must be made in standby mode
	boolean was_active = setStandby();

	wireReadDataByte(REG_XYZ_DATA_CFG, statusCheck);

	if (filter) {
		wireWriteDataByte(REG_HP_FILTER_CUTOFF, (statusCheck | HPFMask));
	} else {
		wireWriteDataByte(REG_HP_FILTER_CUTOFF, (statusCheck & ~HPFMask));
	}

	//active Mode if was active
	if (was_active)
		setActive();
}

/***********************************************************
 *
 * setActive
 *
 *
 *
 ***********************************************************/
void MMA8451_n0m1::setActive() {
	byte statusCheck;

	wireReadDataByte(REG_CTRL_REG1, statusCheck);
//	delay(1);
	wireWriteDataByte(REG_CTRL_REG1, (statusCheck | activeMask));


}

/***********************************************************
 *
 * setStandby
 *
 *
 *
 ***********************************************************/
boolean MMA8451_n0m1::setStandby() {
	byte statusCheck;

	wireReadDataByte(REG_CTRL_REG1, statusCheck);

	if (statusCheck & activeMask) {
		wireWriteDataByte(REG_CTRL_REG1, (statusCheck & ~activeMask));
		return true;
	} else {
		return false;
	}
}

/***********************************************************
 *
 * setLNoise
 *
 *
 *
 ***********************************************************/
void MMA8451_n0m1::setLNoise() {
	byte statusCheck;

	wireReadDataByte(REG_CTRL_REG1, statusCheck);
	wireWriteDataByte(REG_CTRL_REG1, (statusCheck | lowNoiseMask));
}

void MMA8451_n0m1::clearLNoise() {
	byte statusCheck;

	wireReadDataByte(REG_CTRL_REG1, statusCheck);
	wireWriteDataByte(REG_CTRL_REG1, (statusCheck & ~lowNoiseMask));
}

void MMA8451_n0m1::setOFFX(byte off_x) {
	boolean was_active = setStandby();
	wireWriteDataByte(REG_OFF_X, off_x);
	if (was_active)
		setActive();
}

void MMA8451_n0m1::setOFFY(byte off_y) {
	boolean was_active = setStandby();
	wireWriteDataByte(REG_OFF_Y, off_y);
	if (was_active)
		setActive();
}

void MMA8451_n0m1::setOFFZ(byte off_z) {
	boolean was_active = setStandby();
	wireWriteDataByte(REG_OFF_Z, off_z);
	if (was_active)
		setActive();
}

char MMA8451_n0m1::getOFFX() {
	byte statusCheck;
	wireReadDataByte(REG_OFF_X, statusCheck);
	return statusCheck;
}
char MMA8451_n0m1::getOFFY() {
	byte statusCheck;
	wireReadDataByte(REG_OFF_Y, statusCheck);
	return statusCheck;
}
char MMA8451_n0m1::getOFFZ() {
	byte statusCheck;
	wireReadDataByte(REG_OFF_Z, statusCheck);
	return statusCheck;
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
bool MMA8451_n0m1::wireWriteDataByte(uint8_t reg, uint8_t val) {

	if (!i2c_write_reg_8(MMA8451_ADDRESS, reg, val)) {
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
bool MMA8451_n0m1::wireReadDataByte(uint8_t reg, uint8_t &val) {

	if (!i2c_write8(MMA8451_ADDRESS, reg)) {
		//return false;
	}

	// repeated start
	if (!i2c_read8(MMA8451_ADDRESS, &val)) {
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
int MMA8451_n0m1::wireReadDataBlock(uint8_t reg, uint8_t *val, unsigned int len) {

	if (!i2c_write8(MMA8451_ADDRESS, reg)) {
		//return 0;
	}

	// repeated start
	if (!i2c_read_n(MMA8451_ADDRESS, val, len)) {
		return 0;
	}

	return len;
}
