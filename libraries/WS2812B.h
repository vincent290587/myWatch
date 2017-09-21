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

#define WS_BLACK   0,0,0
#define WS_RED     0xFF,0,0
#define WS_GREEN   0,0xFF,0
#define WS_BLUE    0,0,0xFF
#define WS_YELLOW  0xFF,0xFF,0
#define WS_ORANGE  0xFF,0x99,0x33
#define WS_VIOLET  0xFF,0x33,0xFF

typedef struct
{
    uint8_t         max;  /**< Maximum */
    uint8_t         min;  /**< Minimum */
    uint8_t         step; /**< step. */
    uint8_t         rgb[3];
    uint32_t        on_time_ticks;   /**< Ticks to stay in high impulse state. */
} neo_sb_init_params_t;

class WS2812B {
public:
	WS2812B();
	void init(uint8_t pin_num);
	void clear() { neopixel_clear(&strip);}
	void show() {  neopixel_show(&strip);}
	uint8_t setColor(uint8_t red, uint8_t green, uint8_t blue ) {
		return neopixel_set_color(&strip, 0, red, green, blue );
	}

	void setNotify(uint8_t red, uint8_t green, uint8_t blue);
	void setNotify(uint8_t red, uint8_t green, uint8_t blue, uint8_t on_time);
	void setWeakNotify(uint8_t red, uint8_t green, uint8_t blue);
	void setWeakNotify(uint8_t red, uint8_t green, uint8_t blue, uint8_t on_time);
	void run(void);

private:
	neopixel_strip_t strip;
	neo_sb_init_params_t _params;
	bool leds_is_on;     /**< Flag for indicating if LEDs are on. */
	bool is_counting_up; /**< Flag for indicating if counter is incrementing or decrementing. */
	int32_t pause_ticks;
	uint32_t ratio;
};

#endif /* LIBRARIES_WS2812B_H_ */
