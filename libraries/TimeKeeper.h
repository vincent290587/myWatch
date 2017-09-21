/*
 * TimeKeeper.h
 *
 *  Created on: 4 mars 2017
 *      Author: Vincent
 */

#ifndef LIBRARIES_TIMEKEEPER_H_
#define LIBRARIES_TIMEKEEPER_H_

#include "Arduino.h"

extern "C" uint32_t millis();

class TimeKeeper {

public:
	TimeKeeper();
	void increment_100millis();
	void increment_seconds();
	void setTime(uint8_t seconds, uint8_t minutes, uint8_t hours, uint8_t day, uint8_t month, uint16_t year);
	bool isNight();

	uint8_t getDay() const {
		return _day;
	}

	void setDay(uint8_t day) {
		_day = day;
	}

	uint8_t getHours() const {
		return _hours;
	}

	void setHours(uint8_t hours) {
		_hours = hours;
	}

	uint8_t getMinutes() const {
		return _minutes;
	}

	void setMinutes(uint8_t minutes) {
		_minutes = minutes;
	}

	uint8_t getMonth() const {
		return _month;
	}

	void setMonth(uint8_t month) {
		_month = month;
	}

	uint8_t getSeconds() const {
		return _seconds;
	}

	void setSeconds(uint8_t seconds) {
		_seconds = seconds;
	}

	uint16_t getYear() const {
		return _year;
	}

	void setYear(uint16_t year) {
		_year = year;
	}

	uint32_t _millis;
	static TimeKeeper* pTimeKeeper;

private:
	bool _initialized;
	uint8_t _nb_increments;
	uint8_t _seconds, _minutes, _hours, _day, _month;
	uint16_t _year;
};

#endif /* LIBRARIES_TIMEKEEPER_H_ */
