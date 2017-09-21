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

void Vue::printBatt() {

	float cor_volt = stc.getCorrectedVoltage(BATT_INT_RES);
	int perc = percentageBatt(cor_volt, stc.getCurrent());

	// SOC
	sharp.setTextSize(1);
    sharp.setCursor(70, 5);
    sharp.print(perc);

    // charge
    sharp.setTextSize(1);
    sharp.setCursor(70, 15);
    sharp.print(stc.getCharge(), stc.getCharge()*stc.getCharge()>100?0:1);

    // icone
	sharp.fillRect(120, 10, 3, 6, BLACK); // petit bout
	sharp.drawRect(90, 7, 30, 12, BLACK); // forme exterieure
	if (perc > 3) {
		int Blevel = regFenLim(perc, 0., 100., 1., 26.);
		sharp.fillRect(92, 9, Blevel, 8, BLACK);
	}

	// current
	sharp.setTextSize(2);
	sharp.setCursor(5, 5);
	sharp.print(stc.getCurrent(), stc.getCurrent()>10?0:1);

	// Battery voltage
	sharp.setCursor(10, 20);
	sharp.setTextSize(1);
	sharp.print(cor_volt);

	// charge
	if (nrf_gpio_pin_read(6)==HIGH) {
		// VUSB present
		sharp.setCursor(90, 22);
		sharp.setTextSize(1);
		if (nrf_gpio_pin_read(11)==LOW) {
			// charge terminated
			sharp.print("CHRG");
		} else {
			sharp.print("END");
			neopix.setWeakNotify(WS_RED);
		}

	}

}


void Vue::bareMinimum() {

	uint8_t off = 20;

	// time
	sharp.setTextSize(3);
	sharp.setTextColor(BLACK);
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

}

void Vue::DebugScreen() {

	this->printBatt();

	sharp.setCursor(0, 35);
	sharp.setTextSize(1);
	sharp.println(String("Action fxos: ") + String((uint8_t)acc.getLastAction()));
	sharp.println("");
	sharp.println(String("Action adps.: ") + String((uint8_t)adps.getLastAction()));
	sharp.println("");
//	sharp.println(String("Acc.acc_x: ") + String(acc.xg()));
//	sharp.println(String("Acc.acc_y: ") + String(acc.yg()));
//	sharp.println(String("Acc.acc_z: ") + String(acc.zg()));
//	sharp.println("");
//	sharp.println(String("Acc.mag_x: ") + String(acc.getMagX()));
//	sharp.println(String("Acc.mag_y: ") + String(acc.getMagY()));
//	sharp.println("");
	sharp.println(String("VEML ID: ") + String(veml.getDevID()));
	sharp.println(String("VEML Cnf: ") + String(veml.getConf()));
	sharp.println(String("VEML UVA: ") + String(veml.getUVA()));
	sharp.println(String("VEML IVi: ") + String(veml.getUVIndex()));
	sharp.println(String("VEML visible: ") + String(veml.getRawVisComp()));
	sharp.println(String("VEML IR: ") + String(veml.getRawIRComp()));
	sharp.println("");
	//sharp.println(String("VEML visible: ") + String(veml.getRawVisComp()));
	//sharp.println(String("VEML visible: ") + String(veml.getRawVisComp()));
	sharp.println(String("ADPS ON: ") + String(optim.isAdpsAwake()));
	sharp.println(String("LDO ON: ") + String(digitalRead(LED_EN_PIN)));
	sharp.println("");
}

void Vue::VeryLowPowerScreen() {

	this->bareMinimum();

	// Batt
	this->printBatt();

	// temperature
	sharp.setCursor(50, 105);
	sharp.setTextSize(2);
	sharp.print(stc.getTemperature(), 1);
	sharp.print("*C");

}

void Vue::LowPowerScreen() {

	this->bareMinimum();

	// Batt
	this->printBatt();

	// North direction
	uint16_t x1=64, y1=90;
	sharp.drawCircle(64, 105, 15, BLACK);
	rotate_point(acc.computeNorthDirection(), 64, 105, 64, 90, x1, y1);
	sharp.drawLine(64, 105, x1, y1, BLACK);

	// UV
	sharp.fillRect(10, 90, 12, 30, BLACK);
	int UVlevel = regFenLim(veml.getUVIndex(), 0., 15., 28., 1.);
	if (UVlevel > 1) {
		sharp.fillRect(11, 91, 10, UVlevel, WHITE);
	}

	// VISIBLE
	sharp.drawRect(106, 90, 14, 30, BLACK);

	uint16_t vis_light = veml.getRawVisComp();
	uint16_t ir_light = veml.getRawIRComp();
	static float max_light = 100.;
	max_light = MAX(vis_light, max_light);
	max_light = MAX(ir_light, max_light);
	int alevel = regFenLim(ir_light, 0., max_light, 1., 28.);
	int blevel = regFenLim(vis_light, 0., max_light, 1., 28.);
	if (alevel > 1) {
		sharp.fillRect(108, 119-alevel, 4, alevel, BLACK);
	}
	if (blevel > 1) {
		sharp.fillRect(114, 119-blevel, 4, blevel, BLACK);
	}

}


void Vue::SPO2Screen() {

	uint16_t i;
	int16_t min_, max_, fen_;
	static uint32_t last_counter = 0;

	this->printBatt();

	sharp.setTextColor(BLACK);

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
	sharp.setCursor(40, 30);
	sharp.print(spo_hrm.getHR());
	sharp.print(" / ");
	sharp.print(spo_hrm.getSPO2());
}

void Vue::run() {

	if (_needsRefresh && !_isInTrans) {

		// USER STATE MACHINE
		switch (state) {
		default:
		case DEBUG:
			DebugScreen();
			break;

		case VERY_LOW_POWER:
			VeryLowPowerScreen();
			break;

		case LOW_POWER:
			LowPowerScreen();
			break;

		case SPORT:
			SPO2Screen();
			break;
		}

		displayNotification();

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

void Vue::addNotification(uint8_t type_, const char *title_, const char *msg_) {

	notif.timetag = 0;
	notif.isNew = true;

	notif.type = type_;
	notif.title = title_;
	notif.msg = msg_;

}


void Vue::addNotification(SNotif* notif_) {

	if (!notif_) return;

	notif.timetag = 0;
	notif.isNew = true;

	notif.type = notif_->type;
	notif.title = notif_->title;
	notif.msg = notif_->msg;

	//NRF_LOG_INFO("Ajout notif: %s\r\n", (uint32_t)notif.msg.c_str());
}

void Vue::displayNotification() {

	if (notif.isNew == true) {
		notif.timetag = millis();
		notif.isNew = false;
	}

	if (notif.timetag &&
			millis() - notif.timetag < NOTIFICATION_PERSISTENCE) {

		sharp.fillRect(0, 0, LCDWIDTH, LCDHEIGHT / 2+2, WHITE);
		sharp.drawRect(0, 0, LCDWIDTH, LCDHEIGHT / 2, BLACK);
		sharp.setCursor(5, 5);
		sharp.setTextSize(2);
		sharp.setTextColor(CLR_NRM);
		if (notif.type != 0) {
			sharp.print(notif.title);
			sharp.println(":");
		}
		sharp.print(notif.msg);

	}


}
