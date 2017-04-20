/*
 * SPO2HRM.h
 *
 *  Created on: 1 mars 2017
 *      Author: Vincent
 */

#ifndef LIBRARIES_SPO2HRM_H_
#define LIBRARIES_SPO2HRM_H_

#include "RingBuffer.h"
#include "MAX30102.h"
#include "HRFilter.h"
#include "algorithm.h"

#define MIN_SAMPLES 64
#define NB_SAMPLES 128
#define SAMPLES_SHIFTING (NB_SAMPLES/4)

#define INT_PIN_MAX  12

extern "C" void max30102ISR(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t polarity_);

class SPO2HRM {
public:
	SPO2HRM ();

	void init();
	void start_measurement();
	void stop_measurement();
	void run();
	void refreshCalculation ();
	int32_t getSPO2() {return spo2;}
	int32_t getHR() {return heartRate;}

	volatile bool toggle_check;
	static SPO2HRM* pSPO2HRM;
	MAX30102 max30102;

	RingBuffer IRring;
	RingBuffer REDring;

private:
	int32_t spo2; //SPO2 value
	int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
	int32_t heartRate; //heart rate value
	int8_t validHeartRate; //indicator to show if the heart rate calculation is valid

	HRFilter IRFilter;
	HRFilter RedFilter;

	bool measurement_started;
	bool first_measurement;

};


#endif /* LIBRARIES_SPO2HRM_H_ */
