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

#define NOTIFICATION_PERSISTENCE 5000

class SNotif {
  public:
    SNotif() {};

    bool isNew;
    uint8_t type;
    uint32_t timetag;
    String title;
    String msg;
};

class Vue {
public:
	Vue();
	void begin();
	void run();
	void triggerRefresh() {_needsRefresh=true;};
	void performTransition();

	void displayNotification();
	void addNotification(uint8_t type_, const char *title_, const char *msg_);
	void addNotification(SNotif *notif);

	void printBatt();
	void bareMinimum();

	void setTK(TimeKeeper *TimeKeeper);

	void testdrawrect(void);

private:
	SNotif notif;
	TSharpMem sharp;
	TimeKeeper* _timekeeper;
	bool _isInTrans;
	bool _needsRefresh;

	void DebugScreen();
	void VeryLowPowerScreen();
	void LowPowerScreen();
	void SPO2Screen();
};


#endif /* VUE_VUE_H_ */
