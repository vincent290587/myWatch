/*
 * Arduino.h
 *
 *  Created on: 25 févr. 2017
 *      Author: Vincent
 */

#ifndef LIBRARIES_ARDUINO_H_
#define LIBRARIES_ARDUINO_H_

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "pgmspace.h"

#include "compiler_abstraction.h"
#include "nrf_drv_spi.h"
#include "nrf_drv_gpiote.h"
#include "nrf.h"
#include "nrf_delay.h"

#ifdef __cplusplus
extern "C" {
#endif

#define APP_TIMER_PRESCALER            7              /**< Value of the RTC1 PRESCALER register. */


#define boolean uint8_t
#define byte    uint8_t

#define swap(a, b) { int16_t t = a; a = b; b = t; }

#define OUTPUT         0
#define INPUT          1
#define INPUT_PULLUP   2
#define INPUT_PULLDOWN 3

#define LOW 0
#define HIGH 1

#define _BV(bit) (1 << (bit))


//////////    FUNCTIONS

void init_timers_millis();

uint32_t min(uint32_t val1, uint32_t val2);
uint32_t max(uint32_t val1, uint32_t val2);

void delay(uint32_t p_time);

uint32_t millis();

uint32_t micros();

void pinMode(uint8_t p_pin, uint8_t p_mode);

uint32_t digitalRead(uint8_t p_pin);

void digitalWrite(uint8_t p_pin, uint8_t p_mode);

void attachInterrupt(nrf_drv_gpiote_pin_t pin_, nrf_drv_gpiote_evt_handler_t evt_handler);

void detachInterrupt(nrf_drv_gpiote_pin_t pin_);

float compute2Complement(uint8_t msb, uint8_t lsb);

int percentageBatt (float tensionValue);

float regFen(float val_, float b1_i, float b1_f, float b2_i, float b2_f);

float regFenLim(float val_, float b1_i, float b1_f, float b2_i, float b2_f);


#ifdef __cplusplus
}
#endif

#endif /* LIBRARIES_ARDUINO_H_ */
