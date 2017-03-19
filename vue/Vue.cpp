/*
 * Vue.cpp
 *
 *  Created on: 4 mars 2017
 *      Author: Vincent
 */

#include "mathutils.h"
#include "app.h"
#include "Vue.h"
#include "Orbital7pt.h"
#include "mathutils.h"


Vue::Vue() : TSharpMem(VUE_SPI_SS_PIN) {
	_timekeeper = 0;
	this->setFont(&Orbitron_Light_7);
}


void Vue::setTK(TimeKeeper *timekeeper) {
	_timekeeper = timekeeper;
}

void Vue::LowPowerScreen(float current, float northDirection) {

	uint8_t off = 20;

	this->setTextColor(BLACK);

	// current
	this->drawRect(3,3,60,25, BLACK);

	this->setTextSize(2);
	this->setCursor(5, 5);
	this->print(current, 1);

	// time
	this->setTextSize(3);
	this->setCursor(0, 40);
	this->setCursor(this->getCursorX() + off, this->getCursorY());
	this->print(_timekeeper->getHours()<10?String("0"):String(""));
	this->print(_timekeeper->getHours());
	this->print(":");
	this->print(_timekeeper->getMinutes()<10?String("0"):String(""));
	this->println(_timekeeper->getMinutes());
	this->setCursor(this->getCursorX() + off + 50, this->getCursorY() + 3);
	this->print(_timekeeper->getSeconds()<10?String("0"):String(""));
	this->print(_timekeeper->getSeconds());

	// North direction
	uint16_t x1, y1;
	this->drawCircle(64, 105, 15, BLACK);
	rotate_point(northDirection, 64, 105, 64, 90, x1, y1);
	this->drawLine(64, 105, x1, y1, BLACK);



}

