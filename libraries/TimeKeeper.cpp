/*
 * TimeKeeper.cpp
 *
 *  Created on: 4 mars 2017
 *      Author: Vincent
 */

#include "TimeKeeper.h"

#define NRF_LOG_MODULE_NAME "TK"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

TimeKeeper* TimeKeeper::pTimeKeeper = 0;

uint32_t millis() {

	return TimeKeeper::pTimeKeeper->_millis;

}


TimeKeeper::TimeKeeper() {
	pTimeKeeper = this;

	_initialized = false;
	_nb_increments = 0;
	_millis = 0;
	_seconds = 50;
	_minutes = 7;
	_hours = 22;
	_day = 10;
	_month = 0;
	_year = 0;
}

void TimeKeeper::increment_100millis() {

	_nb_increments++;
	if (_nb_increments >= 10) {
		increment_seconds();
		_nb_increments = 0;
	}

	_millis += 100;

}


void TimeKeeper::increment_seconds()
{
    _seconds++;
    if (_seconds > 59)
    {
        _seconds = 0;
        _minutes++;
        if (_minutes > 59)
        {
            _minutes = 0;
            _hours++;
            if (_hours > 23)
            {
                _hours = 0;
                _day++;
                if (_day > 31)
                {
                    _day = 0;
                    _month++;
                    if (_month > 12)
                    {
                        _year++;
                    }
                }
            }
        }
    }
}



void TimeKeeper::setTime(uint8_t seconds, uint8_t minutes, uint8_t hours, uint8_t day, uint8_t month, uint16_t year)
{
	_seconds = seconds;
	_minutes = minutes;
	_hours = hours;
	_day = day;
	_month = month;
	_year = year;


}

bool TimeKeeper::isNight() {

	bool res = true;

	if (_hours > 9) return false;

	return res;
}
