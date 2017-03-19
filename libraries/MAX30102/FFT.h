/*
 * FFT.h
 *
 *  Created on: 13 mars 2017
 *      Author: Vincent
 */

#ifndef LIBRARIES_MAX30102_FFT_H_
#define LIBRARIES_MAX30102_FFT_H_

#include "Arduino.h"

#define FFT_LIB_REV 0x02a
/* Custom constants */
#define FFT_FORWARD 0x01
#define FFT_REVERSE 0x00
/* Windowing type */
#define FFT_WIN_TYP_RECTANGLE 0x00 /* rectangle (Box car) */
#define FFT_WIN_TYP_HAMMING 0x01 /* hamming */
#define FFT_WIN_TYP_HANN 0x02 /* hann */
#define FFT_WIN_TYP_TRIANGLE 0x03 /* triangle (Bartlett) */
#define FFT_WIN_TYP_BLACKMAN 0x04 /* blackmann */
#define FFT_WIN_TYP_FLT_TOP 0x05 /* flat top */
#define FFT_WIN_TYP_WELCH 0x06 /* welch */

class FFT {
public:
	/* Constructor */
	FFT(void);
	/* Destructor */
	~FFT(void);
	/* Functions */
	void ComplexToMagnitude(double *vReal, double *vImag, uint16_t samples);
	void Compute(double *vReal, double *vImag, uint16_t samples, uint8_t dir);
	void Compute(double *vReal, double *vImag, uint16_t samples, uint8_t power, uint8_t dir);
	double MajorPeak(double *vD, uint16_t samples, double samplingFrequency);
	uint8_t Revision(void);
	void Windowing(double *vData, uint16_t samples, uint8_t windowType, uint8_t dir);
	uint8_t Exponent(uint16_t value);

private:
	/* Functions */
	void Swap(double *x, double *y);

};


#endif /* LIBRARIES_MAX30102_FFT_H_ */
