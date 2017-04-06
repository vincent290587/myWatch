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

#define NRF_LOG_MODULE_NAME "VUE"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

Vue::Vue() : TSharpMem(VUE_SPI_SS_PIN) {
	_timekeeper = 0;
	this->setFont(&Orbitron_Light_7);
}


void Vue::setTK(TimeKeeper *timekeeper) {
	_timekeeper = timekeeper;
}

void Vue::LowPowerScreen(float northDirection) {

	uint8_t off = 20;

	this->setTextColor(BLACK);

	// current
	this->drawRect(3,3,60,25, BLACK);

	this->setTextSize(2);
	this->setCursor(5, 5);
	this->print(_current, 1);

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


void Vue::SPO2Screen(SPO2HRM *spo2) {

	uint16_t i;
	int16_t min_, max_, fen_;
	static uint32_t last_counter = 0;

	this->setTextColor(BLACK);

	// current
	this->drawRect(3,3,60,25, BLACK);

	this->setTextSize(2);
	this->setCursor(5, 5);
	this->print(_current, 1);

	uint16_t x, y;
	const uint8_t MEDIANE = 100;

	min_ = spo2->IRring.getMinValue();
	max_ = spo2->IRring.getMaxValue();

	fen_ = MAX(max_, -min_);
	fen_ = MIN(fen_, 4000);
	fen_ = MAX(fen_, 500);

	NRF_LOG_INFO("Graph min/max %d/%d-%d\r\n", min_, max_, fen_);
	NRF_LOG_ERROR("NB data acq: %u\r\n", spo2->IRring._tot_data - last_counter);
	last_counter = spo2->IRring._tot_data;

	for (i=0; i < spo2->IRring._nb_data && i < LCDWIDTH; i++) {

		x = i;
		y = regFenLim(spo2->IRring.geti(i), -fen_, fen_, 2*MEDIANE - LCDHEIGHT, LCDHEIGHT-5);

		this->drawPixel(x,y,BLACK);
		this->drawPixel(x,y+1,BLACK);
	}

	// HR + SPO2
	this->setTextSize(2);
	this->setCursor(5+70, 5);
	this->print(spo2->getHR());
	this->print(" / ");
	this->print(spo2->getSPO2());
}
