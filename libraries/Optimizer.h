/*
 * Optimizer.h
 *
 *  Created on: 25 avr. 2017
 *      Author: Vincent
 */

#ifndef LIBRARIES_OPTIMIZER_H_
#define LIBRARIES_OPTIMIZER_H_

#include "Arduino.h"


class Optimizer {
public:
	Optimizer(void);

	void wakeADPS(void);
	bool isAdpsAwake(void);
	void enableLDO();
	void shutDownLDO();
	void shutdownADPS();
	void run(void);

private:
	uint32_t lastAdpsWake;
	uint8_t isNightprev;
};



#endif /* LIBRARIES_OPTIMIZER_H_ */
