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

#include "Serial.h"

#include "ADPS9960.h"
#include "MAX30102.h"
#include "MS5637.h"
#include "FXOS8700.h"
#include "SPO2HRM.h"
#include "STC3100.h"
#include "VEML6075.h"
#include "WS2812B.h"

#include "app.h"


#define RUN_DELAY                      100
#define APP_DELAY                      APP_TIMER_TICKS(RUN_DELAY, APP_TIMER_PRESCALER) // ticks from ms
APP_TIMER_DEF(m_app_timer);

//const nrf_drv_timer_t TIMER_APP = NRF_DRV_TIMER_INSTANCE(1);


APP* APP::pApp = 0;


//MMA8451_n0m1 acc;
FXOS8700 acc;
MS5637 baro;
Serial serial;
APDS9960 adps;
SPO2HRM spo_hrm;
STC3100 stc;
VEML6075 veml;
WS2812B neopix(20);

//static void app_timeout_handler(nrf_timer_event_t event_type, void* p_context) {
////static void app_timeout_handler(void* p_context) {
//
//	APP::pApp->bigTick = true;
//	APP::pApp->timekeeper.increment_100millis();
//}

static void dummy_timeout_handler(void* p_context) {
	APP::pApp->bigTick = true;
	APP::pApp->timekeeper.increment_100millis();
}

void init_timers_millis() {

	// Create security request timer.
	ret_code_t ret = app_timer_create(&m_app_timer, APP_TIMER_MODE_REPEATED, dummy_timeout_handler);
	APP_ERROR_CHECK(ret);

	ret = app_timer_start(m_app_timer, APP_DELAY, NULL);
	APP_ERROR_CHECK(ret);


//	uint32_t time_ticks, err_code;
//	nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
//
//	err_code = nrf_drv_timer_init(&TIMER_APP, &timer_cfg, app_timeout_handler);
//	APP_ERROR_CHECK(err_code);
//
//	time_ticks = nrf_drv_timer_ms_to_ticks(&TIMER_APP, RUN_DELAY);
//
//	nrf_drv_timer_extended_compare( &TIMER_APP, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
//
//	nrf_drv_timer_enable(&TIMER_APP);

}

APP::APP() {
	pApp = this;
	bigTick = false;
	_state = SPORT;
}



void APP::init() {

	NRF_LOG_INFO("Init\r\n");

	init_timers_millis();

	// light init
	neopix.clear();

	////////////////////////////////////////////////////

	// spi init
	spi_init();
	nrf_delay_ms(1);

	// screen init
	vue.clearDisplay();
	vue.setTK(&timekeeper);
	vue.begin();


	////////////////////////////////////////////////////

	// i2c init
	i2c_init();

	// STC3100 init
//	stc.init();

	// accelero configuration
//	acc.init();

	// motion sensing init
//	adps.init();
	//adps.disablePower();

	// baro init
//	baro.init();

	// UV init
//	veml.init();

	// MAX30102 init
	spo_hrm.init();
//	spo_hrm.max30102.shutDown();
	spo_hrm.start_measurement();
}


void APP::run() {


}

void APP::run_low_power() {

	neopix.setColor(0xFF, 0, 0);

	if (0) {

		if (nbTicks == 1) {

			NRF_LOG_INFO("Voltage: " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(stc.getVoltage()));
			NRF_LOG_INFO("Temperature: " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(stc.getTemperature()));
			NRF_LOG_ERROR("Power consumption: " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(stc.getCurrent()));
		}
		else if (nbTicks == 2) {

			acc.update();
			NRF_LOG_INFO("Xg: " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(acc.xg()));
			NRF_LOG_INFO("Yg: " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(acc.yg()));
			NRF_LOG_INFO("Zg: " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(acc.zg()));

			vue.LowPowerScreen(acc.computeNorthDirection());

		}
		else if (nbTicks == 3) {

			veml.poll();
			NRF_LOG_INFO("UVA: " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(veml.getUVA()));
			NRF_LOG_INFO("UVB: " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(veml.getUVB()));

		}
	}

	acc.update();

	// update neopixel
	neopix.show();

}

void APP::run_very_low_power() {

}

void APP::run_sport() {

	if (nbTicks%2 == 1) spo_hrm.run();

	if (nbTicks == 1) {

		//NRF_LOG_INFO("Voltage: " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(stc.getVoltage()));
		//NRF_LOG_INFO("Temperature: " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(stc.getTemperature()));
		//NRF_LOG_ERROR("Power consumption: " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(stc.getCurrent()));

		if (digitalRead(INT_PIN_MAX) == LOW) {
			NRF_LOG_WARNING("Pin MAX low\r\n");
		} else {
			NRF_LOG_WARNING("Pin MAX high\r\n");
		}

	} else if (nbTicks == 5) {

		spo_hrm.refreshCalculation();
		vue.SPO2Screen(&spo_hrm);

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

		if (!nbTicks) {

			// update current
			if (stc.isPresent(STC3100_ADDRESS)) {
				stc.refresh();
				vue.setCurrent(stc.getCurrent());
			} else {
				vue.setCurrent(-1);
			}

			// force display all
			vue.refresh();

			// reset screen
			vue.resetBuffer();
		} else {
			// USER STATE MACHINE
			switch (_state) {
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
		}
	}
	// functions needed to run continuously
	//runADPS();

}

void APP::runADPS() {

	static uint8_t todo = false;

	if (!todo && (adps.gestureISRFlag == true || digitalRead(2) == LOW)) {
		if (adps.isGestureAvailable()) {
			NRF_LOG_ERROR("Gesture available ;-) !\r\n");
			todo = true;
			adps.gestureISRFlag = false;
		}
	}

	if (todo) {
		switch ( adps.readGesture() ) {
		case DIR_UP:
			todo = false;
			NRF_LOG_ERROR("UP");
			break;
		case DIR_DOWN:
			todo = false;
			NRF_LOG_ERROR("DOWN");
			break;
		case DIR_LEFT:
			todo = false;
			NRF_LOG_ERROR("LEFT");
			break;
		case DIR_RIGHT:
			todo = false;
			NRF_LOG_ERROR("RIGHT");
			break;
		case DIR_NEAR:
			todo = false;
			NRF_LOG_ERROR("NEAR");
			break;
		case DIR_FAR:
			todo = false;
			NRF_LOG_ERROR("FAR");
			break;
		case DIR_NONE:
			todo = false;
			NRF_LOG_ERROR("NONE");
			break;
		default:
			break;

		}

	}
}


void APP::testdrawrect(void) {
	for (uint8_t i = 0; i < vue.height() / 2; i += 2) {
		vue.drawRect(i, i, vue.width() - 2 * i, vue.height() - 2 * i, BLACK);
		vue.refresh();
	}
 }

