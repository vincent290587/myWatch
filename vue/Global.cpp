/*
 * Global.cpp
 *
 *  Created on: 9 avr. 2017
 *      Author: Vincent
 */

#include "Global.h"

Serial serial;

APP app;

namespace mvc {
//MMA8451_n0m1 acc;
FXOS8700 acc;
MS5637 baro;
APDS9960 adps;
SPO2HRM spo_hrm;
STC3100 stc;
VEML6075 veml;
WS2812B neopix;

State prev_state;
State state;
Optimizer optim;

Vue vue;

Controller control;
}

Global::Global() {
	// TODO Auto-generated constructor stub

}

