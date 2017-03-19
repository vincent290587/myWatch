/*
 * Serial.cpp

 *
 *  Created on: 26 févr. 2017
 *      Author: Vincent
 */

#include <string.h>

#define NRF_LOG_MODULE_NAME "Serial"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include "Serial.h"

Serial::Serial() {
	name = 0;
}

Serial::Serial(char *name_) {
	name = name_;
}

void Serial::print(uint32_t number) {

	NRF_LOG_RAW_INFO("%u", number);

}

void Serial::print(int number) {

	NRF_LOG_RAW_INFO("%d", number);

}

void Serial::print(float number) {

	NRF_LOG_RAW_INFO("" NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(number));

}

void Serial::print(const char *string_) {

	NRF_LOG_RAW_INFO("%s", (uint32_t)string_);

}

void Serial::println(void) {

	NRF_LOG_RAW_INFO("\r\n");

}

void Serial::println(uint32_t number) {

	NRF_LOG_RAW_INFO("%u\r\n", number);

}

void Serial::println(int number) {

	NRF_LOG_RAW_INFO("%d\r\n", number);

}

void Serial::println(float number) {

	NRF_LOG_RAW_INFO("" NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(number));

}

void Serial::println(const char *string_) {

	NRF_LOG_RAW_INFO("%s\r\n", (uint32_t)string_);

}

