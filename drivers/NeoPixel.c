/* Lava
 *
 * WS2812B Tricolor LED (neopixel) controller
 *
 *
 * Example code:

	neopixel_strip_t m_strip;
	uint8_t dig_pin_num = 6;
	uint8_t leds_per_strip = 24;
	uint8_t error;
	uint8_t led_to_enable = 10;
	uint8_t red = 255;
	uint8_t green = 0;
	uint8_t blue = 159;

	neopixel_init(&m_strip, dig_pin_num, leds_per_strip);
	neopixel_clear(&m_strip);
	error = neopixel_set_color_and_show(&m_strip, led_to_enable, red, green, blue);
	if (error) {
		//led_to_enable was not within number leds_per_strip
	}
	//clear and remove strip
	neopixel_clear(&m_strip);
	neopixel_destroy(&m_strip);


 * For use with BLE stack, see information below:
	- Include in main.c
		#include "ble_radio_notification.h"
	- Call (see nrf_soc.h: NRF_RADIO_NOTIFICATION_DISTANCES and NRF_APP_PRIORITIES)
		ble_radio_notification_init(NRF_APP_PRIORITY_xxx,
									NRF_RADIO_NOTIFICATION_DISTANCE_xxx,
									your_radio_callback_handler);
	- Create
		void your_radio_callback_handler(bool radio_active)
		{
			if (radio_active == false)
			{
				neopixel_show(&strip1);
				neopixel_show(&strip2);
				//...etc
			}
		}
	- Do not use neopixel_set_color_and_show(...) with BLE, instead use uint8_t neopixel_set_color(...);
 */

#include "NeoPixel.h"

#include "nrf_delay.h"
#include "nrf_gpio.h"


//These defines are timed specific to a series of if statements and will need to be changed
//to compensate for different writing algorithms than the one in neopixel.c

// nop=62.5ns
#define NEOPIXEL_SEND_ONE	NRF_GPIO->OUTSET = (1UL << PIN); \
		__ASM ( \
				" NOP\n\t" \
				" NOP\n\t" \
				" NOP\n\t" \
				" NOP\n\t" \
				" NOP\n\t" \
				" NOP\n\t" \
				" NOP\n\t" \
				" NOP\n\t" \
		); \
		NRF_GPIO->OUTCLR = (1UL << PIN); \
		__ASM ( \
				" NOP\n\t" \
		);

// modif pour avoir le bon ON-time
#define NEOPIXEL_SEND_ZERO \
	    NRF_GPIO->OUTSET = (1UL << PIN); \
		NRF_GPIO->OUTCLR = (1UL << PIN);  \
		__ASM ( \
				" NOP\n\t" \
				" NOP\n\t" \
				" NOP\n\t" \
				" NOP\n\t" \
				" NOP\n\t" \
				" NOP\n\t" \
				" NOP\n\t" \
				" NOP\n\t" \
				" NOP\n\t" \
		);


////////////       FUNCTIONS

void neopixel_init(neopixel_strip_t *strip, uint8_t pin_num, uint16_t num_leds)
{
	strip->pin_num = pin_num;
	strip->num_leds = num_leds;

	nrf_gpio_cfg_output(pin_num);
	NRF_GPIO->OUTCLR = (1UL << pin_num);

	for (int i = 0; i < num_leds; i++)
	{
		strip->leds[i].simple.g = 0;
		strip->leds[i].simple.r = 0;
		strip->leds[i].simple.b = 0;
	}
}

void neopixel_clear(neopixel_strip_t *strip)
{
	for (int i = 0; i < strip->num_leds; i++)
	{
		strip->leds[i].simple.g = 0;
		strip->leds[i].simple.r = 0;
		strip->leds[i].simple.b = 0;
	}
	neopixel_show(strip);
}

#include "nrf_nvic.h"
void neopixel_show(neopixel_strip_t *strip)
{
	const uint8_t PIN =  strip->pin_num;

	__disable_irq();
	//uint8_t is_nested_critical_region=0;
	//sd_nvic_critical_region_enter(&is_nested_critical_region);

	NRF_GPIO->OUTCLR = (1UL << PIN);
	nrf_delay_us(55);

	for (int i = 0; i < strip->num_leds; i++) {

		for (int j = 0; j < 3; j++) {

			if ((strip->leds[i].grb[j] & 128) > 0)	{NEOPIXEL_SEND_ONE}
			else	{NEOPIXEL_SEND_ZERO}

//			NEOPIXEL_COMPLETE_CYCLE

			if ((strip->leds[i].grb[j] & 64) > 0)	{NEOPIXEL_SEND_ONE}
			else	{NEOPIXEL_SEND_ZERO}

//			NEOPIXEL_COMPLETE_CYCLE

			if ((strip->leds[i].grb[j] & 32) > 0)	{NEOPIXEL_SEND_ONE}
			else	{NEOPIXEL_SEND_ZERO}

//			NEOPIXEL_COMPLETE_CYCLE

			if ((strip->leds[i].grb[j] & 16) > 0)	{NEOPIXEL_SEND_ONE}
			else	{NEOPIXEL_SEND_ZERO}

//			NEOPIXEL_COMPLETE_CYCLE

			if ((strip->leds[i].grb[j] & 8) > 0)	{NEOPIXEL_SEND_ONE}
			else	{NEOPIXEL_SEND_ZERO}

//			NEOPIXEL_COMPLETE_CYCLE

			if ((strip->leds[i].grb[j] & 4) > 0)	{NEOPIXEL_SEND_ONE}
			else	{NEOPIXEL_SEND_ZERO}

//			NEOPIXEL_COMPLETE_CYCLE

			if ((strip->leds[i].grb[j] & 2) > 0)	{NEOPIXEL_SEND_ONE}
			else	{NEOPIXEL_SEND_ZERO}

//			NEOPIXEL_COMPLETE_CYCLE

			if ((strip->leds[i].grb[j] & 1) > 0)	{NEOPIXEL_SEND_ONE}
			else	{NEOPIXEL_SEND_ZERO}

//			NEOPIXEL_COMPLETE_CYCLE
		}
	}

	__enable_irq();
	//sd_nvic_critical_region_exit(is_nested_critical_region);
}

uint8_t neopixel_set_color(neopixel_strip_t *strip, uint16_t index, uint8_t red, uint8_t green, uint8_t blue )
{
	if (index < strip->num_leds)
	{
		strip->leds[index].simple.r = red;
		strip->leds[index].simple.g = green;
		strip->leds[index].simple.b = blue;
	} else {
		return 1;
	}
	return 0;
}

uint8_t neopixel_set_color_and_show(neopixel_strip_t *strip, uint16_t index, uint8_t red, uint8_t green, uint8_t blue)
{
	if (index < strip->num_leds)
	{
		strip->leds[index].simple.r = red;
		strip->leds[index].simple.g = green;
		strip->leds[index].simple.b = blue;
		neopixel_show(strip);
	} else {
		return 1;
	}

	return 0;
}


