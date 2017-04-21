/*
 * app.cpp
 *
 *  Created on: 25 févr. 2017
 *      Author: Vincent
 */

#include "Arduino.h"

#include "app_timer.h"
#include "nrf_drv_timer.h"
#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include "I2C.h"
#include "SPI.h"

#include "app.h"
#include "Global.h"

#define LED_EN_PIN                     1

#define RUN_DELAY                      100
#define APP_DELAY                      APP_TIMER_TICKS(RUN_DELAY, APP_TIMER_PRESCALER) // ticks from ms
APP_TIMER_DEF(m_app_timer);


APP* APP::pApp = 0;


using namespace mvc;


static void dummy_timeout_handler(void* p_context) {
	APP::pApp->bigTick = true;
	APP::pApp->timekeeper.increment_100millis();
}

void init_timers_millis() {

	ret_code_t ret = app_timer_create(&m_app_timer, APP_TIMER_MODE_REPEATED, dummy_timeout_handler);
	APP_ERROR_CHECK(ret);

	ret = app_timer_start(m_app_timer, APP_DELAY, NULL);
	APP_ERROR_CHECK(ret);
}

APP::APP() {
	pApp = this;
	bigTick = false;
	state = LOW_POWER;
}



void APP::init() {

	NRF_LOG_INFO("Init\r\n");

	init_timers_millis();

	// light init
	neopix.setColor(0, 0, 0);
	neopix.clear();
// TODO remove
#ifndef NRF51
	return;
#endif
	////////////////////////////////////////////////////

	// spi init
	spi_init();
	nrf_delay_ms(1);

	// screen init
	vue.setTK(&timekeeper);
	vue.begin();


	////////////////////////////////////////////////////

	// i2c init
	i2c_init();

	// STC3100 init
	stc.init();

	// accelero configuration
	acc.init();

	// init LDO 2
	nrf_gpio_cfg_output(LED_EN_PIN);
	nrf_gpio_pin_clear(LED_EN_PIN);

	// motion sensing init
	adps.init();
	adps.disablePower();
	//adps.enableGestureSensor(true);

	// baro init
	baro.init();

	// UV init
	veml.init();

	// MAX30102 init
//	nrf_gpio_pin_set(LED_EN_PIN);
	spo_hrm.init();
	spo_hrm.max30102.shutDown();
//	spo_hrm.start_measurement();
}


void APP::run() {

	// functions needed to run continuously:
    adps.run();

	// run state machine (10Hz)
	sm_run();

}

void APP::switchMode(uint8_t new_mode) {

	switch (state) {
	case LOW_POWER:
		if (new_mode==SPORT) {
			nrf_gpio_pin_set(LED_EN_PIN);
			spo_hrm.start_measurement();

			// mode change
			state = SPORT;
		}
		break;
	case SPORT:
		if (new_mode==LOW_POWER) {
			spo_hrm.stop_measurement();
			spo_hrm.max30102.shutDown();
			nrf_gpio_pin_clear(LED_EN_PIN);

			// mode change
			state = LOW_POWER;
		}
		break;
	}


}

void APP::run_very_low_power() {

}


void APP::run_low_power() {

	if (nbTicks == 1) {

		NRF_LOG_INFO("Voltage: " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(stc.getVoltage()));
		NRF_LOG_INFO("Temperature: " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(stc.getTemperature()));
		NRF_LOG_ERROR("Power consumption: " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(stc.getCurrent()));
	}
	else if (nbTicks == 2) {

		NRF_LOG_INFO("Xg: " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(acc.xg()));
		NRF_LOG_INFO("Yg: " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(acc.yg()));
		NRF_LOG_INFO("Zg: " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(acc.zg()));

	}
	else if (nbTicks == 3) {

		NRF_LOG_INFO("UVA: " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(veml.getUVA()));
		NRF_LOG_INFO("UVB: " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(veml.getUVB()));

	}

	// refresh at 3Hz
	if ((nbTicks%3) == 0) {
		vue.triggerRefresh();
	}

	// TODO run at 10Hz without interrupts
	acc.update();

}

void APP::run_sport() {

	// Gather MAX30102 data
	if ((nbTicks%2) == 1) spo_hrm.run();

	if (nbTicks == 1) {

		NRF_LOG_ERROR("Power consumption: " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(stc.getCurrent()));

		if (digitalRead(INT_PIN_MAX) == LOW) {
			NRF_LOG_WARNING("Pin MAX low\r\n");
			spo_hrm.run();
		}

	} else if (nbTicks == 5) {

		spo_hrm.refreshCalculation();

		// refresh screen
		vue.triggerRefresh();

	}

}


void APP::sm_run() {

	// app management
	if (bigTick == true) {

		// bigtick reset
		bigTick = false;

		// tick increase
		nbTicks++;
		nbTicks = nbTicks % 10;

		// run controller
		control.run();
		// run neopixel
		neopix.run();

		if (!nbTicks) {

			// update current
			if (stc.isPresent(STC3100_ADDRESS)) {
				stc.refresh();
			}

			// update UV
			veml.poll();

			// trigger a refresh at least once per second
			vue.triggerRefresh();
		}

		// USER STATE MACHINE
		switch (state) {
		case VERY_LOW_POWER:
			this->run_very_low_power();
			break;
		case LOW_POWER:
			this->run_low_power();
			break;
		case SPORT:
			this->run_sport();
			break;
		}

		// display
		vue.run();

	}

}
