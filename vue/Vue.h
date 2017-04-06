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
#include "SPO2HRM.h"
#include "app.h"

#define VUE_SPI_SS_PIN 4


class Vue : public TSharpMem {
public:
	Vue();

	void setTK(TimeKeeper *TimeKeeper);
	void setCurrent(float current) {_current = current;}

	void LowPowerScreen(float);
	void SPO2Screen(SPO2HRM*);

private:
	TimeKeeper* _timekeeper;
	float _current;
};


#endif /* VUE_VUE_H_ */
