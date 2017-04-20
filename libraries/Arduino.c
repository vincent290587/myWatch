/*
 * Arduino.c
 *
 *  Created on: 25 févr. 2017
 *      Author: Vincent
 */

#include "math.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "app_util_platform.h"
#include "app_timer.h"
#include "app_error.h"
#include "nordic_common.h"
#include "sdk_config.h"

#define NRF_LOG_MODULE_NAME "ARDU"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include "Arduino.h"


#define TICKS_TO_MS(ticks)                                                           \
		(                                                                            \
				ticks * ( (APP_TIMER_PRESCALER + 1 ) * 1000 ) / APP_TIMER_CLOCK_FREQ \
		)

#define TICKS_TO_US(ticks)                                                           \
		(                                                                            \
				ticks * ( (APP_TIMER_PRESCALER + 1 ) * 1000 ) / APP_TIMER_CLOCK_FREQ \
		)



uint32_t min(uint32_t val1, uint32_t val2) {
	if (val1 < val2) {
		return val1;
	} else {
		return val2;
	}
}

uint32_t max(uint32_t val1, uint32_t val2) {
	if (val1 < val2) {
		return val2;
	} else {
		return val1;
	}
}

void delay(uint32_t p_time) {

	nrf_delay_ms(p_time);

}

// fonction externalisee au timekeeper
//uint32_t millis() {
//
//	static uint32_t nb_rollover = 0;
//	static uint32_t last_count = 0;
//	uint32_t cur_count = app_timer_cnt_get();
//
//	if (cur_count < last_count) {
//		nb_rollover += 1;
//		NRF_LOG_ERROR("Rollover !\r\n");
//	}
//
//	uint32_t rtc1_freq = APP_TIMER_CLOCK_FREQ / (APP_TIMER_PRESCALER + 1 );
//
//	uint64_t millis = cur_count;
//	millis *= 1000;
//	millis /= rtc1_freq;
//
//	//NRF_LOG_INFO("Elapsed counts: %u\r\n", cur_count);
//
//	millis += (nb_rollover * 0xFFFFFF / rtc1_freq) * 1000;
//
//	last_count = cur_count;
//
//	return (uint32_t)millis;
//}

uint32_t micros() {

	static uint32_t nb_rollover = 0;
	static uint32_t last_count = 0;
	uint32_t cur_count = app_timer_cnt_get();

	if (cur_count < last_count) {
		nb_rollover += 1;
		NRF_LOG_ERROR("Rollover !\r\n");
	}

	uint32_t rtc1_freq = APP_TIMER_CLOCK_FREQ / (APP_TIMER_PRESCALER + 1 );

	uint64_t micros = cur_count;
	micros *= 100;
	micros /= rtc1_freq;

	//NRF_LOG_INFO("Elapsed counts: %u\r\n", cur_count);

	micros += (nb_rollover * 0xFFFFFF / rtc1_freq) * 100;

	last_count = cur_count;

	return (uint32_t)micros;
}

void pinMode(uint8_t p_pin, uint8_t p_mode) {

	if (p_mode == OUTPUT) {
		nrf_gpio_cfg_output(p_pin);
	} else {
		switch (p_mode) {
		case INPUT:
			nrf_gpio_cfg_input(p_pin, NRF_GPIO_PIN_NOPULL);
			break;
		case INPUT_PULLDOWN:
			nrf_gpio_cfg_input(p_pin, NRF_GPIO_PIN_PULLDOWN);
			break;
		case INPUT_PULLUP:
			nrf_gpio_cfg_input(p_pin, NRF_GPIO_PIN_PULLUP);
			break;
		default:
			NRF_LOG_ERROR("Wrong pin configuration");
			break;
		}
	}
}

void digitalWrite(uint8_t p_pin, uint8_t p_mode) {
	if (p_mode == LOW) {
		nrf_gpio_pin_clear(p_pin);
	} else {
		nrf_gpio_pin_set(p_pin);
	}
}

uint32_t digitalRead(uint8_t p_pin) {
	return nrf_gpio_pin_read(p_pin);
}

void attachInterrupt(nrf_drv_gpiote_pin_t pin_, nrf_drv_gpiote_evt_handler_t evt_handler) {

	ret_code_t err_code;

	if (!nrf_drv_gpiote_is_init()) {
		err_code = nrf_drv_gpiote_init();
		APP_ERROR_CHECK(err_code);
	}

	nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
	in_config.hi_accuracy = false;
	in_config.is_watcher = false;
	in_config.sense = NRF_GPIOTE_POLARITY_HITOLO;
	in_config.pull = NRF_GPIO_PIN_PULLUP;

	err_code = nrf_drv_gpiote_in_init(pin_, &in_config, evt_handler);
	APP_ERROR_CHECK(err_code);

	nrf_drv_gpiote_in_event_enable(pin_, true);
}

void detachInterrupt(nrf_drv_gpiote_pin_t pin_) {

	nrf_drv_gpiote_in_uninit(pin_);

}

float compute2Complement(uint8_t msb, uint8_t lsb) {
	uint16_t t;
	uint16_t val;
	uint8_t tl=lsb, th=msb;
	float ret;

	if (th & 0b00100000) {
		t = th << 8;
		val = (t & 0xFF00) | (tl & 0x00FF);
		val -= 1;
		val = ~(val | 0b1110000000000000);
		//NRF_LOG_INFO("Raw 2c1: %u\r\n", val);
		ret = (float)val;
	} else {
		t = (th & 0xFF) << 8;
		val = (t & 0xFF00) | (tl & 0x00FF);
		//NRF_LOG_INFO("Raw 2c2: %u\r\n", val);
		ret = (float)-val;
	}

	return ret;
}

int percentageBatt (float tensionValue) {

  float fp_ = 0.;

  if (tensionValue > 3.78) {
    fp_ = 536.34*tensionValue*tensionValue*tensionValue-6723.8*tensionValue*tensionValue;
    fp_ += 28186*tensionValue-39402;
  } else if (tensionValue > 2.) {
    fp_ = pow(10, -11.4)*pow(tensionValue, 22.315);
  } else {
    fp_ = 0;
  }

  return (int)fp_;
}

//float min(float val1, float val2) {
//  if (val1 <= val2) return val1;
//  else return val2;
//}
//
//float max(float val1, float val2) {
//  if (val1 <= val2) return val2;
//  else return val1;
//}

float regFen(float val_, float b1_i, float b1_f, float b2_i, float b2_f) {

  float x, res;
  // calcul x
  x = (val_ - b1_i) / (b1_f - b1_i);

  // calcul valeur: x commun
  res = x * (b2_f - b2_i) + b2_i;
  return res;
}

float regFenLim(float val_, float b1_i, float b1_f, float b2_i, float b2_f) {

  float x, res;
  // calcul x
  x = (val_ - b1_i) / (b1_f - b1_i);

  // calcul valeur: x commun
  res = x * (b2_f - b2_i) + b2_i;
  if (res < min(b2_i,b2_f)) res = min(b2_i,b2_f);
  if (res > max(b2_i,b2_f)) res = max(b2_i,b2_f);
  return res;
}

