/*
 * app.h
 *
 *  Created on: 25 f�vr. 2017
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
	uint32_t nbTicks;
	uint8_t app_mode;

	void testdrawrect(void);
};

#endif /* APP_APP_H_ */
