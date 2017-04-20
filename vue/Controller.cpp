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
}

void Controller::run() {

	// update internal variables
	acc.runController();
	adps.runController();

	Action act = acc.getAction();
	Direction dir = acc.getDirection();

	switch (act) {
	case STAP:
		break;
	case DTAP:
		break;
	case SHAKE:
		// TODO create an auto disable
		adps.enablePower();
		adps.enableGestureSensor(true);
		break;
	case SWIPE:
	case NONE:
	default:
		break;
	}

	act = adps.getAction();
	dir = adps.getDirection();

	if (act == SWIPE) {
		switch (dir) {
		case Xp:
			app.switchMode(SPORT);
			break;
		case Xm:
			app.switchMode(LOW_POWER);
			break;
		default:
			break;
		}
	}

}
