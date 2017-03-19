/*
 * WS2812B.h
 *
 * Wrapper for the neopixel LED
 *
 *  Created on: 28 févr. 2017
 *      Author: Vincent
 */

#ifndef LIBRARIES_WS2812B_H_
#define LIBRARIES_WS2812B_H_

#include "Arduino.h"
#include "NeoPixel.h"

class WS2812B {
public:
	WS2812B(uint8_t pin_num) {
		neopixel_init(&strip, pin_num, 1);
	}

	void clear() {
		neopixel_clear(&strip);
	}
	void show() {
		neopixel_show(&strip);
	}
	uint8_t setColor(uint8_t red, uint8_t green, uint8_t blue ) {
		return neopixel_set_color(&strip, 0, red, green, blue );
	}

private:
	neopixel_strip_t strip;
};

#endif /* LIBRARIES_WS2812B_H_ */
