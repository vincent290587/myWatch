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

#include "Global.h"
#include "app.h"


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
	state = DEBUG;
}



void APP::init() {

	NRF_LOG_INFO("Init\r\n");

	init_timers_millis();

	// light init
	neopix.init(24);
	neopix.clear();

	// v_usb pres
	nrf_gpio_cfg_input(6, NRF_GPIO_PIN_NOPULL);
	// end of charge
	nrf_gpio_cfg_input(11, NRF_GPIO_PIN_PULLUP);


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

#ifdef NRF52
	return;
#endif

	// STC3100 init
	stc.init(40, STC3100_MODE_ULTRAHIGHRES);

	// accelero configuration
	acc.init();

	// init LDO 2
	nrf_gpio_cfg_output(LED_EN_PIN);
	optim.enableLDO();
	//delay(10);

	// motion sensing init
	adps.init();
	//adps.enableLightSensor(false);

	// baro init
	baro.init();

	// UV init
	veml.init();

	// MAX30102 init
	spo_hrm.init();
	spo_hrm.max30102.shutDown();

	vue.addNotification(1, "DEBUG", "Init");
}


void APP::run() {

	// functions needing to run continuously:
	adps.run();

	if (state == SPORT) {
		spo_hrm.run();
	}

	acc.run();

	// run state machine (10Hz)
	sm_run();

}

// state transitions
void APP::switchMode(uint8_t new_mode) {

	if (State(new_mode) == state) return;

	if (State(new_mode) == VERY_LOW_POWER) {

		veml.off();

	} else if (state == VERY_LOW_POWER) {

		// on quitte le mode VERY_LOW_POWER
		veml.on();

	}

	if (State(new_mode) == VERY_LOW_POWER || State(new_mode) == LOW_POWER) {

		if (state == SPORT) {

			spo_hrm.stop_measurement();

		}

	} else if (State(new_mode) == SPORT) {
		// new mode is SPORT
		spo_hrm.start_measurement();
	}

	state = State(new_mode);

}

void APP::run_very_low_power() {

	// run at XHz without interrupts
	if (nbTicks == 5) acc.update();

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

	// run at XHz without interrupts
	acc.update();

	// refresh at 2Hz
	if ((nbTicks%5) == 0) {
		vue.triggerRefresh();
	}

}

void APP::run_sport() {

	// Gather MAX30102 data
	//if ((nbTicks%2) == 1) spo_hrm.run();

	if (nbTicks == 1) {

		NRF_LOG_ERROR("Power consumption: " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(stc.getCurrent()));

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
		nbTicks = nbTicks % 10;

		// power optimizer
		optim.run();
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
		default:
		case DEBUG:
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

		nbTicks++;
	}

}
