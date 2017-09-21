/*
 * Global.h
 *
 *  Created on: 9 avr. 2017
 *      Author: Vincent
 */

#ifndef VUE_GLOBAL_H_
#define VUE_GLOBAL_H_

#include "ADPS9960.h"
#include "MAX30102.h"
#include "MS5637.h"
#include "FXOS8700.h"
#include "SPO2HRM.h"
#include "STC3100.h"
#include "VEML6075.h"
#include "WS2812B.h"
#include "Serial.h"
#include "Optimizer.h"

#include "app.h"

#include "Vue.h"
#include "Controller.h"

#define LED_EN_PIN                     1

#define BATT_INT_RES                   0.433

enum State { DEBUG, VERY_LOW_POWER , LOW_POWER , SPORT };

extern Serial serial;

extern APP app;

namespace mvc {
//extern MMA8451_n0m1 acc;
extern FXOS8700 acc;
extern MS5637 baro;
extern APDS9960 adps;
extern SPO2HRM spo_hrm;
extern STC3100 stc;
extern VEML6075 veml;
extern WS2812B neopix;

extern State prev_state;
extern State state;
extern Optimizer optim;

extern Vue vue;

extern Controller control;
}


class Global {
public:
	Global();
};

#endif /* VUE_GLOBAL_H_ */
