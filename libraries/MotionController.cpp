/*
 * MotionController.cpp
 *
 *  Created on: 6 avr. 2017
 *      Author: Vincent
 */

#include <MotionController.h>
#include "Global.h"

using namespace mvc;

MotionController::MotionController() {

	_last_stap_time = 0;
	_last_stap_dir = Or;
	_nb_single_tap = 0;

	_last_action = NONE;
	_last_action_dir = Or;
	_last_action_time = 0;

	_action_der = 0;

}

void MotionController::notify(Action act_, Direction dir_) {

	// update action count
	_action_der += 1;
	if (_action_der > MAX_DERIVEE) {
		return;
	}

	// debounce
	if ((millis() - _last_action_time < DEBOUNCE_TIME)) return;

	switch (act_) {
	case STAP:
		// need to wait to detect double taps
		if (_last_stap_time && _nb_single_tap == 1 &&
				(millis() - _last_stap_time > 50) &&
				(millis() - _last_stap_time < WINDOW_TIME)) {
			// double tap
			_last_action = DTAP;
			// TODO determiner la vraie direction
			_last_action_dir = dir_ & _last_action_dir;
			_last_action_time = millis();

			//vue.addNotification(1, "FXOS", "STAP => DTAP");

			neopix.setNotify(WS_YELLOW, 1);
			// reset
			_nb_single_tap = 0;
			_last_stap_time = 0;
			_last_stap_dir = Or;
		} else {
			// note the first tap timestamp
			_nb_single_tap++;
			_last_action_dir = dir_;
			// we take the tap time
			_last_stap_time = millis();
		}
		break;

	case DTAP:
		_last_action = DTAP;
		_last_action_dir = dir_;
		_last_action_time = millis();

		//vue.addNotification(1, "FXOS", "True DTAP");

		neopix.setNotify(WS_VIOLET, 4);
		break;

	case SHAKE:
		_last_action = SHAKE;
		_last_action_dir = dir_;
		_last_action_time = millis();

		neopix.setNotify(WS_YELLOW, 1);
		break;

	case SWIPE:
		_last_action = SWIPE;
		_last_action_dir = dir_;
		_last_action_time = millis();

		neopix.setNotify(WS_ORANGE, 1);
		break;

	case NONE:
	default:
		break;
	}
}

// must be called only once !!
Action MotionController::getAction() {
	Action res = _last_action;
	if (_last_action != NONE) {
		_last_action_perm = _last_action;
	}
	_last_action = NONE;
	return res;
}

// Must be called only once !!
Direction MotionController::getDirection() {
	Direction res = Direction(_last_action_dir);
	_last_action_dir = Or;
	return res;
}

// can be called infinitely
Action MotionController::getLastAction() {
	Action res = NONE;

	if (millis() - _last_action_time <= 2000) {
		res = _last_action_perm;
	}

	return res;
}

// needs to run at 10Hz
void MotionController::runController(void) {

	// update action count
	if (_action_der > 0) _action_der -= 1;

	// update single taps
	if (_last_stap_time && _nb_single_tap == 1 &&
			(millis() - _last_stap_time > WINDOW_TIME)) {
		// single tap detected
		_last_action = STAP;
		_last_action_dir = _last_stap_dir;
		_last_action_time = millis();

		vue.addNotification(1, "FXOS", "STAP");

		neopix.setNotify(WS_YELLOW, 1);
		// reset
		_nb_single_tap = 0;
		_last_stap_time = 0;
		_last_stap_dir = Or;

	} else if ((_last_stap_time || _nb_single_tap) &&
			(millis() - _last_stap_time > WINDOW_TIME)) {
		// false detection
		// -> reset
		_nb_single_tap = 0;
		_last_stap_time = 0;
		_last_stap_dir = Or;
	}


}
