/*
 * MotionController.h
 *
 *  Created on: 6 avr. 2017
 *      Author: Vincent
 */

#ifndef LIBRARIES_MOTIONCONTROLLER_H_
#define LIBRARIES_MOTIONCONTROLLER_H_

#include "Arduino.h"
#include "TimeKeeper.h"

#define MAX_DERIVEE  4
#define WINDOW_TIME 350
#define DEBOUNCE_TIME 50

typedef enum {
	NONE=0,
	STAP=1,
	DTAP=2,
	SHAKE=3,
	SWIPE=4
} Action;

typedef enum {
	Or=0,
	Xp=1<<0,
	Xm=1<<1,
	Yp=1<<2,
	Ym=1<<3,
	Zp=1<<4,
	Zm=1<<5
} Direction;

class MotionController {
public:
	MotionController();

	void notify(Action, Direction);

	Action getAction();
	Direction getDirection();
	Action getLastAction();

	void runController(void);


private:
	uint32_t _last_stap_time;
	uint8_t _last_stap_dir;
	uint8_t _nb_single_tap;

	Action _last_action;
	Action _last_action_perm;
	uint8_t _last_action_dir;
	uint32_t _last_action_time;

	int16_t _action_der;
};

#endif /* LIBRARIES_MOTIONCONTROLLER_H_ */
