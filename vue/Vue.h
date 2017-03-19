/*
 * Vue.h
 *
 *  Created on: 4 mars 2017
 *      Author: Vincent
 */

#ifndef VUE_VUE_H_
#define VUE_VUE_H_

#include "TimeKeeper.h"
#include "LS013B7DH03.h"
#include "app.h"

#define VUE_SPI_SS_PIN 4


class Vue : public TSharpMem {
public:
	Vue();

	void setTK(TimeKeeper *TimeKeeper);

	void LowPowerScreen(float, float);

private:
	TimeKeeper* _timekeeper;
};


#endif /* VUE_VUE_H_ */
