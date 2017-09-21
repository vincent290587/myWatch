/*
 * WS2812B.cpp
 *
 * Neopixel LED
 *
 *  Created on: 20 april. 2017
 *      Author: Vincent
 */


#include "WS2812B.h"


#define ON_STEPS_NB    5
#define ON_TICKS_DEFAULT 3


WS2812B::WS2812B() {

	_params.max = 10;
	_params.min = 0;

	_params.rgb[0] = 0;
	_params.rgb[1] = 0;
	_params.rgb[2] = 0;

	_params.step = _params.max / ON_STEPS_NB;
	_params.on_time_ticks = ON_TICKS_DEFAULT;

	is_counting_up = true;

}

void WS2812B::init(uint8_t pin_num) {

	neopixel_init(&strip, pin_num, 1);

}

void WS2812B::setNotify(uint8_t red, uint8_t green, uint8_t blue) {

	_params.rgb[0] = red;
	_params.rgb[1] = green;
	_params.rgb[2] = blue;

	// start process
	is_counting_up = true;
	leds_is_on  = true;
	ratio       = _params.min;
	pause_ticks = 0;

	_params.on_time_ticks = ON_TICKS_DEFAULT;
}

void WS2812B::setNotify(uint8_t red, uint8_t green, uint8_t blue, uint8_t on_time) {

	this->setNotify(red, green, blue);
	_params.on_time_ticks = on_time;

}

void WS2812B::setWeakNotify(uint8_t red, uint8_t green, uint8_t blue) {

	if (!leds_is_on) {
		this->setNotify(red, green, blue);
		_params.on_time_ticks = ON_TICKS_DEFAULT;
	}

}

void WS2812B::setWeakNotify(uint8_t red, uint8_t green, uint8_t blue, uint8_t on_time) {

	if (!leds_is_on) {
		this->setNotify(red, green, blue);
		_params.on_time_ticks = on_time;
	}

}

void WS2812B::run() {

	if (!leds_is_on) {
		this->clear();
		return;
	}
	
	// continue process
    if (pause_ticks <= 0)
    {
        if (is_counting_up)
        {
            if (int(ratio) >= int(_params.max - _params.step))
            {
                // start decrementing.
                is_counting_up = false;
                pause_ticks = _params.on_time_ticks;
            }
            else
            {
                ratio += _params.step;
            }
        }
        else
        {
            if (int(ratio) <= int(_params.min + _params.step))
            {
                // Min is reached, we are done
            	// end process
            	leds_is_on = false;
            	this->clear();
            	return;
            }
            else
            {
                ratio -= _params.step;
            }
        }
    }
    else
    {
        pause_ticks -= 1;
    }

    // update neopixel
    this->setColor(_params.rgb[0] * ratio * ratio / 255,
    		 _params.rgb[1] * ratio * ratio / 255,
			 _params.rgb[2] * ratio * ratio / 255);
    this->show();
}



