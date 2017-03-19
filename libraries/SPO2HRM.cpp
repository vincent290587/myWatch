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

	measurement_started = false;
	first_measurement = true;
	samples_acquired = 0;
	toggle_check = false;
}

void SPO2HRM::init() {
	max30102.init();
}

void SPO2HRM::start_measurement() {
	measurement_started = true;

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

	int1 = max30102.getINT1();

	if (int1 & 0b10000) {
		// proximity INT: a finger just arrived
		toggle_check = false;
		// reset the samples acquired
		samples_acquired = 0;
		max30102.resetAvailable();
	} else if (int1 & 0b10000000) {
		// FIFO INT
		toggle_check = true;
	} else {
		toggle_check = false;
		delay(10);
	}

	if (toggle_check) {
		NRF_LOG_INFO("Checking MAX FIFO buffers after ISR %u\r\n", int1);
		max30102.check();
		toggle_check = false;

		// samples already received but not treated
		new_samples = max30102.available();

		if (!new_samples) {
			delay(5);
			return;
		}

		for (uint8_t i=0; i < new_samples && samples_acquired < NB_SAMPLES;i++) {
			_red_buffer[samples_acquired] = max30102.getFIFORed();
			_ir_buffer[samples_acquired] = max30102.getFIFOIR();
			samples_acquired++;
			max30102.nextSample();
		}

		if (samples_acquired >= NB_SAMPLES) {
			// stop measurmeent

			NRF_LOG_INFO("Samples processing\r\n");
			NRF_LOG_FLUSH();

			first_measurement = false;

			// start algorithm
			//After gathering 25 new samples recalculate HR and SP02
			maxim_heart_rate_and_oxygen_saturation(_ir_buffer, samples_acquired, _red_buffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

			NRF_LOG_ERROR("HRM: %d  valid=%d\r\n", heartRate, validHeartRate);

			// reset to 75
			samples_acquired -= SAMPLES_SHIFTING;

			//dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
			for (uint8_t i = SAMPLES_SHIFTING; i < 100; i++) {
				_red_buffer[i - SAMPLES_SHIFTING] = _red_buffer[i];
				_ir_buffer[i - SAMPLES_SHIFTING] = _ir_buffer[i];
			}

		} else {
			NRF_LOG_INFO("%u Samples to go\r\n", NB_SAMPLES - samples_acquired);
		}


	}
}
