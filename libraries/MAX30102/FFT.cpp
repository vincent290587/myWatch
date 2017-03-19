/*
 * FFT.cpp
 *
 *  Created on: 13 mars 2017
 *      Author: Vincent
 */


#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "FFT.h"
#include "mathutils.h"

#define twoPi 6.28318531
#define fourPi 12.56637061

FFT::FFT(void)
{
	/* Constructor */
}

FFT::~FFT(void)
{
	/* Destructor */
}

uint8_t FFT::Revision(void)
{
	return(FFT_LIB_REV);
}

void FFT::Compute(double *vReal, double *vImag, uint16_t samples, uint8_t dir)
{
	Compute(vReal, vImag, samples, Exponent(samples), dir);
}

void FFT::Compute(double *vReal, double *vImag, uint16_t samples, uint8_t power, uint8_t dir)
{
	/* Computes in-place complex-to-complex FFT */
	/* Reverse bits */
	uint16_t j = 0;
	for (uint16_t i = 0; i < (samples - 1); i++) {
		if (i < j) {
			Swap(&vReal[i], &vReal[j]);
			Swap(&vImag[i], &vImag[j]);
		}
		uint16_t k = (samples >> 1);
		while (k <= j) {
			j -= k;
			k >>= 1;
		}
		j += k;
	}
	/* Compute the FFT  */
	double c1 = -1.0;
	double c2 = 0.0;
	uint8_t l2 = 1;
	for (uint8_t l = 0; (l < power); l++) {
		uint8_t l1 = l2;
		l2 <<= 1;
		double u1 = 1.0;
		double u2 = 0.0;
		for (j = 0; j < l1; j++) {
			for (uint16_t i = j; i < samples; i += l2) {
				uint16_t i1 = i + l1;
				double t1 = u1 * vReal[i1] - u2 * vImag[i1];
				double t2 = u1 * vImag[i1] + u2 * vReal[i1];
				vReal[i1] = vReal[i] - t1;
				vImag[i1] = vImag[i] - t2;
				vReal[i] += t1;
				vImag[i] += t2;
			}
			double z = ((u1 * c1) - (u2 * c2));
			u2 = ((u1 * c2) + (u2 * c1));
			u1 = z;
		}
		c2 = sqrt((1.0 - c1) / 2.0);
		if (dir == FFT_FORWARD) {
			c2 = -c2;
		}
		c1 = sqrt((1.0 + c1) / 2.0);
	}
	/* Scaling for reverse transform */
	if (dir != FFT_FORWARD) {
		for (uint16_t i = 0; i < samples; i++) {
			vReal[i] /= samples;
			vImag[i] /= samples;
		}
	}
}

void FFT::ComplexToMagnitude(double *vReal, double *vImag, uint16_t samples)
{
	/* vM is half the size of vReal and vImag */
	for (uint8_t i = 0; i < samples; i++) {
		vReal[i] = sqrt(sq(vReal[i]) + sq(vImag[i]));
	}
}

void FFT::Windowing(double *vData, uint16_t samples, uint8_t windowType, uint8_t dir)
{
	/* Weighing factors are computed once before multiple use of FFT */
	/* The weighing function is symetric; half the weighs are recorded */
	double samplesMinusOne = (double(samples) - 1.0);
	for (uint16_t i = 0; i < (samples >> 1); i++) {
		double indexMinusOne = double(i);
		double ratio = (indexMinusOne / samplesMinusOne);
		double weighingFactor = 1.0;
		/* Compute and record weighting factor */
		switch (windowType) {
		case FFT_WIN_TYP_RECTANGLE: /* rectangle (box car) */
			weighingFactor = 1.0;
			break;
		case FFT_WIN_TYP_HAMMING: /* hamming */
			weighingFactor = 0.54 - (0.46 * cos(twoPi * ratio));
			break;
		case FFT_WIN_TYP_HANN: /* hann */
			weighingFactor = 0.54 * (1.0 - cos(twoPi * ratio));
			break;
		case FFT_WIN_TYP_TRIANGLE: /* triangle (Bartlett) */
			weighingFactor = 1.0 - ((2.0 * abs(indexMinusOne - (samplesMinusOne / 2.0))) / samplesMinusOne);
			break;
		case FFT_WIN_TYP_BLACKMAN: /* blackmann */
			weighingFactor = 0.42323 - (0.49755 * (cos(twoPi * ratio))) + (0.07922 * (cos(fourPi * ratio)));
			break;
		case FFT_WIN_TYP_FLT_TOP: /* flat top */
			weighingFactor = 0.2810639 - (0.5208972 * cos(twoPi * ratio)) + (0.1980399 * cos(fourPi * ratio));
			break;
		case FFT_WIN_TYP_WELCH: /* welch */
			weighingFactor = 1.0 - sq((indexMinusOne - samplesMinusOne / 2.0) / (samplesMinusOne / 2.0));
			break;
		}
		if (dir == FFT_FORWARD) {
			vData[i] *= weighingFactor;
			vData[samples - (i + 1)] *= weighingFactor;
		}
		else {
			vData[i] /= weighingFactor;
			vData[samples - (i + 1)] /= weighingFactor;
		}
	}
}

double FFT::MajorPeak(double *vD, uint16_t samples, double samplingFrequency)
{
	double maxY = 0;
	uint16_t IndexOfMaxY = 0;
	for (uint16_t i = 1; i < ((samples >> 1) - 1); i++) {
		if ((vD[i-1] < vD[i]) && (vD[i] > vD[i+1])) {
			if (vD[i] > maxY) {
				maxY = vD[i];
				IndexOfMaxY = i;
			}
		}
	}
	double delta = 0.5 * ((vD[IndexOfMaxY-1] - vD[IndexOfMaxY+1]) / (vD[IndexOfMaxY-1] - (2.0 * vD[IndexOfMaxY]) + vD[IndexOfMaxY+1]));
	double interpolatedX = ((IndexOfMaxY + delta)  * samplingFrequency) / (samples-1);
	/* retuned value: interpolated frequency peak apex */
	return(interpolatedX);
}

/* Private functions */

void FFT::Swap(double *x, double *y)
{
	double temp = *x;
	*x = *y;
	*y = temp;
}

uint8_t FFT::Exponent(uint16_t value)
{
	/* Computes the Exponent of a powered 2 value */
	uint8_t result = 0;
	while (((value >> result) & 1) != 1) result++;
	return(result);
}
