/*
 * SPI.h
 *
 *  Created on: 25 févr. 2017
 *      Author: Vincent
 */

#ifndef DRIVERS_SPI_H_
#define DRIVERS_SPI_H_

#include "nrf_gpio.h"
#include "app_error.h"
#include "boards.h"

#ifdef __cplusplus
extern "C" {
#endif


void spi_init();

void spi_reset_buffer();

void spi_add_byte(uint8_t data_);

void spi_transfer();


#ifdef __cplusplus
}
#endif

#endif /* DRIVERS_SPI_H_ */
