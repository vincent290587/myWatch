/*
 * Controller.cpp
 *
 *  Created on: 9 avr. 2017
 *      Author: Vincent
 */

#include "Controller.h"
#include "Global.h"

using namespace mvc;


Controller::Controller() {
	lastActionTime = 0;
}

void Controller::run() {

	// update internal variables
	acc.runController();

	Action act = acc.getAction();
	Direction dir = acc.getDirection();

	switch (act) {
	case STAP:
		break;
	case DTAP:
		break;
	case SHAKE:
		// create an auto disable
		optim.wakeADPS();
		lastActionTime = millis();
		break;
	case SWIPE:
	case NONE:
	default:
		break;
	}

	adps.runController();

	if (optim.isAdpsAwake()) {

		act = adps.getAction();
		dir = adps.getDirection();

		if (act == SWIPE) {
			switch (dir) {
			case Xp:
				if (state < SPORT) {
					app.switchMode(state+1);
					lastActionTime = millis();
				}
				break;
			case Xm:
				if (state > DEBUG) {
					app.switchMode(state-1);
					lastActionTime = millis();
				}
				break;
			default:
				break;
			}
		}
	}

}
