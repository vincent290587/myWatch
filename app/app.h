/*
 * app.h
 *
 *  Created on: 25 févr. 2017
 *      Author: Vincent
 */

#ifndef APP_APP_H_
#define APP_APP_H_

#include "TimeKeeper.h"
#include "Vue.h"

enum State { VERY_LOW_POWER , LOW_POWER , SPORT };

void init_timers_millis();

class APP {

public:
	APP();
	void init();
	void sm_run();
	void run_very_low_power();
	void run();
	void run_low_power();
	void run_sport();
	void runADPS();

	TimeKeeper timekeeper;
	volatile bool bigTick;
	static APP* pApp;

private:
	Vue vue;
	uint32_t nbTicks;
	uint8_t app_mode;
	State _state;

	void testdrawrect(void);
};

#endif /* APP_APP_H_ */
