/*
 * SPO2HRM.cpp
 *
 *  Created on: 1 mars 2017
 *      Author: Vincent
 */

#include "SPO2HRM.h"

#define NRF_LOG_MODULE_NAME "SPO2"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

SPO2HRM* SPO2HRM::pSPO2HRM = 0;

/***********************************************************
 *
 * accelPulseISR
 *
 *
 *
 ***********************************************************/
void max30102ISR(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t polarity_) {
	// TODO check
	if (polarity_ == NRF_GPIOTE_POLARITY_HITOLO) {
		NRF_LOG_WARNING("Max30102 ISR");
		SPO2HRM::pSPO2HRM->toggle_check = true;
	}
}


SPO2HRM::SPO2HRM() {
	pSPO2HRM = this;

	IRring.reset();
	REDring.reset();

	measurement_started = false;
	toggle_check = false;
}

void SPO2HRM::init() {
	max30102.init();
}

void SPO2HRM::start_measurement() {
	measurement_started = true;

	IRring.reset();
	REDring.reset();

	max30102.wakeUp();
	delay(1);
	//max30102.setup();
	max30102.clearFIFO();

	NRF_LOG_ERROR("Measurement started\r\n");

	// attach interrupt for I2C automated reading
	attachInterrupt(INT_PIN_MAX, max30102ISR);
}

void SPO2HRM::stop_measurement() {
	measurement_started = false;
	// detach interrupt for I2C automated reading
	NRF_LOG_ERROR("Measurement stopped\r\n");
	detachInterrupt(INT_PIN_MAX);
}

void SPO2HRM::run() {

	uint8_t new_samples;
	uint8_t int1 = 0;

	if (!measurement_started) return;

	// TODO remove when interrupt pin is connected
	int1 = max30102.getINT1();
	if (int1 & 0b10000) {
		// proximity INT: a finger just arrived
		toggle_check = false;
		// reset the ring buffers
		IRring.reset();
		REDring.reset();
		// reset filters
		IRFilter.resetFilter();
		RedFilter.resetFilter();
		// reset the samples acquired
		max30102.resetAvailable();
		//max30102.clearFIFO();
	} else if (int1 & 0b10000000) {
		// FIFO INT
		toggle_check = true;
	} else {
		toggle_check = false;
	}
	// end TODO

	if (toggle_check) {
		NRF_LOG_INFO("Checking MAX FIFO buffers after ISR %u\r\n", int1);
		max30102.check();
		toggle_check = false;

		// samples already received but not treated
		new_samples = max30102.available();
		if (!new_samples) {
			return;
		}

		for (uint8_t i=0; i < new_samples;i++) {
			IRring.add(int16_t(IRFilter.filter_cheb1(max30102.getFIFOIR())));
			REDring.add(int16_t(RedFilter.filter_cheb1(max30102.getFIFORed())));

			max30102.nextSample();
		}

	}
}

void SPO2HRM::refreshCalculation () {

	if (IRring._nb_data >= MIN_SAMPLES) {

		NRF_LOG_INFO("Samples processing\r\n");

		// TODO start algorithm
		//maxim_heart	_rate_and_oxygen_saturation(_ir_buffer, IRring._nb_data, _red_buffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
		heartRate = getHRM(&IRring);

		//NRF_LOG_ERROR("SPO2: %d  valid=%u\r\n", spo2, validSPO2);

	} else {
		NRF_LOG_INFO("Not enough samples\r\n");
	}

}
