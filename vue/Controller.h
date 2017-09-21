/*
 * Controller.h
 *
 *  Created on: 9 avr. 2017
 *      Author: Vincent
 */

#ifndef VUE_CONTROLLER_H_
#define VUE_CONTROLLER_H_

#include "Arduino.h"
#include "MotionController.h"

class Controller {
public:
	Controller();
	void run();

	uint32_t getLastActionTime() const {
		return lastActionTime;
	}

private:
	uint32_t lastActionTime=0;
};


#endif /* VUE_CONTROLLER_H_ */
