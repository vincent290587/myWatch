/*
 * NeoPixel.h
 *
 *  Created on: 28 f�vr. 2017
 *      Author: Vincent
 */

#ifndef DRIVERS_NEOPIXEL_H_
#define DRIVERS_NEOPIXEL_H_

#include "Arduino.h"

#ifdef __cplusplus
extern "C" {
#endif


typedef union {
		struct {
			uint8_t g, r, b;
		}simple;
    uint8_t grb[3];
} color_t;

typedef struct {
	uint8_t pin_num;
	uint16_t num_leds;
	color_t leds[2];
} neopixel_strip_t;

/**
  @brief Initialize GPIO and data location
  @param[in] pointer to Strip structure
	@param[in] pin number for GPIO
*/
void neopixel_init(neopixel_strip_t *strip, uint8_t pin_num, uint16_t num_leds);

/**
  @brief Turn all LEDs off
  @param[in] pointer to Strip structure
*/
void neopixel_clear(neopixel_strip_t *strip);

/**
  @brief Update strip with structure data
  @param[in] pointer to Strip structure
*/
void neopixel_show(neopixel_strip_t *strip);

/**
  @brief Write RGB value to LED structure
  @param[in] pointer to Strip structure
	@param[in] red value
	@param[in] green value
	@param[in] blue value
	@param[in] LED number (starting at 1)
  @retval 0 Successful write
  @retval 1 LED number is out of bounds
*/
uint8_t neopixel_set_color(neopixel_strip_t *strip, uint16_t index, uint8_t red, uint8_t green, uint8_t blue );


/**
  @brief Write RGB value to LED structure and update LED
  @param[in] pointer to Strip structure
	@param[in] red value
	@param[in] green value
	@param[in] blue value
	@param[in] LED number (starting at 1)
  @retval 0 Successful write
  @retval 1 LED number is out of bounds
*/
uint8_t neopixel_set_color_and_show(neopixel_strip_t *strip, uint16_t index, uint8_t red, uint8_t green, uint8_t blue);



#ifdef __cplusplus
}
#endif

#endif /* DRIVERS_NEOPIXEL_H_ */
