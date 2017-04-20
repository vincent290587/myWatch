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


class Vue {
public:
	Vue();
	void begin();
	void run();
	void triggerRefresh() {_needsRefresh=true;};
	void performTransition();

	void setTK(TimeKeeper *TimeKeeper);

	void testdrawrect(void);

private:
	TSharpMem sharp;
	TimeKeeper* _timekeeper;
	bool _isInTrans;
	bool _needsRefresh;

	void LowPowerScreen();
	void SPO2Screen();
};


#endif /* VUE_VUE_H_ */
