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
#include "Global.h"

#define NRF_LOG_MODULE_NAME "VUE"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

using namespace mvc;

Vue::Vue() {
	_timekeeper = 0;
	_isInTrans = false;
	_needsRefresh = false;
	sharp.setFont(&Orbitron_Light_7);
}


void Vue::setTK(TimeKeeper *timekeeper) {
	_timekeeper = timekeeper;
}

void Vue::LowPowerScreen() {

	uint8_t off = 20;

	sharp.setTextColor(BLACK);

	// current
	sharp.drawRect(3,3,60,25, BLACK);

	sharp.setTextSize(2);
	sharp.setCursor(5, 5);
	sharp.print(stc.getCurrent(), 1);

	// time
	sharp.setTextSize(3);
	sharp.setCursor(0, 40);
	sharp.setCursor(sharp.getCursorX() + off, sharp.getCursorY());
	sharp.print(_timekeeper->getHours()<10?String("0"):String(""));
	sharp.print(_timekeeper->getHours());
	sharp.print(":");
	sharp.print(_timekeeper->getMinutes()<10?String("0"):String(""));
	sharp.println(_timekeeper->getMinutes());
	sharp.setCursor(sharp.getCursorX() + off + 50, sharp.getCursorY() + 3);
	sharp.print(_timekeeper->getSeconds()<10?String("0"):String(""));
	sharp.print(_timekeeper->getSeconds());

	// North direction
	uint16_t x1=64, y1=90;
	sharp.drawCircle(64, 105, 15, BLACK);
	rotate_point(acc.computeNorthDirection(), 64, 105, 64, 90, x1, y1);
	sharp.drawLine(64, 105, x1, y1, BLACK);

}


void Vue::SPO2Screen() {

	uint16_t i;
	int16_t min_, max_, fen_;
	static uint32_t last_counter = 0;

	sharp.setTextColor(BLACK);

	// current
	sharp.drawRect(3,3,60,25, BLACK);

	sharp.setTextSize(2);
	sharp.setCursor(5, 5);
	sharp.print(stc.getCurrent(), 1);

	uint16_t x, y;
	const uint8_t MEDIANE = 100;

	min_ = spo_hrm.IRring.getMinValue();
	max_ = spo_hrm.IRring.getMaxValue();

	fen_ = MAX(max_, -min_);
	fen_ = MIN(fen_, 4000);
	fen_ = MAX(fen_, 500);

	NRF_LOG_INFO("Graph min/max %d/%d-%d\r\n", min_, max_, fen_);
	NRF_LOG_ERROR("NB data acq: %u\r\n", spo_hrm.IRring._tot_data - last_counter);
	last_counter = spo_hrm.IRring._tot_data;

	for (i=0; i < spo_hrm.IRring._nb_data && i < LCDWIDTH; i++) {

		x = i;
		y = regFenLim(spo_hrm.IRring.geti(i), -fen_, fen_, 2*MEDIANE - LCDHEIGHT, LCDHEIGHT-5);

		sharp.drawPixel(x,y,BLACK);
		sharp.drawPixel(x,y+1,BLACK);
	}

	// HR + SPO2
	sharp.setTextSize(2);
	sharp.setCursor(5+70, 5);
	sharp.print(spo_hrm.getHR());
	sharp.print(" / ");
	sharp.print(spo_hrm.getSPO2());
}

void Vue::run() {

	if (_needsRefresh && !_isInTrans) {

		// USER STATE MACHINE
		switch (state) {
		case VERY_LOW_POWER:
			// TODO
			break;

		case LOW_POWER:
			LowPowerScreen();
			break;

		case SPORT:
			SPO2Screen();
			break;
		}

		// refresh the hardware
		sharp.refresh();

		// buffer is cleared after each display
		sharp.resetBuffer();

		_needsRefresh = false;

	} else if (_needsRefresh) {
		// update the transition
		performTransition();
	}

}

void Vue::begin() {

	sharp.clearDisplay();
	sharp.begin();

}



void Vue::testdrawrect(void) {
	for (uint8_t i = 0; i < sharp.height() / 2; i += 2) {
		sharp.drawRect(i, i, sharp.width() - 2 * i, sharp.height() - 2 * i, BLACK);
		sharp.refresh();
	}
}

void Vue::performTransition() {
	// TODO
	_needsRefresh = false;
	_isInTrans = false;
}
