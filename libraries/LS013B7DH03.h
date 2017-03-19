/*
 * LS013B7DH03.h
 *
 *  Created on: 25 févr. 2017
 *      Author: Vincent
 */

#ifndef LIBRARIES_LS013B7DH03_H_
#define LIBRARIES_LS013B7DH03_H_

#include "Arduino.h"
#include "Adafruit_GFX.h"
#include "SPI.h"

#define adagfxswap(a, b) { int16_t t = a; a = b; b = t; }

// LCD Dimensions
#define SHARPMEM_LCDWIDTH       (128)
#define SHARPMEM_LCDHEIGHT      (128)

#define CLR_INV WHITE, BLACK
#define CLR_NRM BLACK, WHITE

#define LCDWIDTH       SHARPMEM_LCDHEIGHT
#define LCDHEIGHT      SHARPMEM_LCDWIDTH

#define BLACK 0
#define WHITE 1

class TSharpMem: public Adafruit_GFX {
public:
	TSharpMem(uint8_t ss = 4);
	void begin(void);
	void drawPixel(int16_t x, int16_t y, uint16_t color);
	uint8_t getPixel(uint16_t x, uint16_t y);
	void clearDisplay(void);

	void resetBuffer(void);
	void writeWhole(void);
	void refresh() {writeWhole();}

private:
	uint8_t _ss;
	uint8_t _sharpmem_vcom;

	void sendbyte(uint8_t data);
	void sendbyteLSB(uint8_t data);
	void sendbyteLSB_last(uint8_t data);
protected:
	uint8_t pcs_data, pcs_command;

	void writedata8_cont(uint8_t c) __attribute__((always_inline)) {
		spi_add_byte(c);
	}
	void writedata8_last(uint8_t c) __attribute__((always_inline)) {
		spi_add_byte(c);
		spi_transfer();
		//spi_reset_buffer();
	}
};

#endif /* LIBRARIES_LS013B7DH03_H_ */
