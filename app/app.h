/*
 * app.h
 *
 *  Created on: 25 févr. 2017
 *      Author: Vincent
 */

#ifndef APP_APP_H_
#define APP_APP_H_

#include "TimeKeeper.h"

void init_timers_millis();

class APP {

public:
	APP();
	void init();
	void run();
	void switchMode(uint8_t);

	TimeKeeper timekeeper;
	volatile bool bigTick;
	static APP* pApp;

private:
	uint32_t nbTicks;
	uint8_t app_mode;

	void sm_run();
	void run_low_power();
	void run_sport();
	void run_very_low_power();
};

#endif /* APP_APP_H_ */
