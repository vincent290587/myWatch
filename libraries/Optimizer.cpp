/*
 * Optimizer.cpp
 *
 *  Created on: 25 avr. 2017
 *      Author: Vincent
 */

#include "Optimizer.h"
#include "nrf_gpio.h"
#include "ble_advertising.h"
#include "TimeKeeper.h"
#include "Global.h"

using namespace mvc;

#define MAX_POWER_ON_TIME  20000

Optimizer::Optimizer()
{
	// awake by default
	lastAdpsWake = 1;
}

void Optimizer::wakeADPS() {

	if (!this->isAdpsAwake()) {

		lastAdpsWake = millis();

		this->enableLDO();
		delay(1);

		// TODO check
		//adps.enablePower();
		if (!adps.enableGestureSensor(true)) {
			APP_ERROR_CHECK(0x18);
		}

	}

}

void Optimizer::shutdownADPS() {

	lastAdpsWake = 0;

	if (!adps.setMode(ALL, OFF)) {
		APP_ERROR_CHECK(0x19);
	}

	// check
	adps.disablePower();

	// manage LDO
	this->shutDownLDO();

}

void Optimizer::run(void) {

	// night management
	if (app.timekeeper.isNight()) {

		if (!isNightprev) {
			// la nuit vient de tomber
			this->shutdownADPS();
		}

		if (state != VERY_LOW_POWER &&
				(!control.getLastActionTime() || millis() - control.getLastActionTime() > 5*60*1000)) {
			app.switchMode(VERY_LOW_POWER);
		}

		isNightprev = 1;
	} else {
		if (isNightprev) {
			// le soleil se leve
			uint32_t ret = ble_advertising_start(BLE_ADV_MODE_SLOW);
			APP_ERROR_CHECK(ret);
		}

//		if (state == VERY_LOW_POWER &&
//				(!control.getLastActionTime() || millis() - control.getLastActionTime() > 5*60*1000)) {
//			app.switchMode(LOW_POWER);
//		}

		isNightprev = 0;
	}

	// ADPS power management
	if (lastAdpsWake &&
			(millis() - lastAdpsWake >= MAX_POWER_ON_TIME)) {

		this->shutdownADPS();

	} else if (lastAdpsWake) {

		this->enableLDO();

	}

}

bool Optimizer::isAdpsAwake(void) {

	// ADPS power management
	if (lastAdpsWake &&
			(millis() - lastAdpsWake < MAX_POWER_ON_TIME)) {

		return 1;

	} else {

		return 0;

	}
}

void Optimizer::enableLDO() {
	// enable LDO
	nrf_gpio_pin_set(LED_EN_PIN);
}

void Optimizer::shutDownLDO() {

	if (state==SPORT || this->isAdpsAwake()) return;

	// shutdown LDO
	nrf_gpio_pin_clear(LED_EN_PIN);
}
