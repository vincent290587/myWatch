/*
 * RingBuffer.h
 *
 *  Created on: 2 avr. 2017
 *      Author: Vincent
 */

#ifndef LIBRARIES_RBUFFER_RINGBUFFER_H_
#define LIBRARIES_RBUFFER_RINGBUFFER_H_

#include "Arduino.h"

#define RBUFFER_SIZE 128

class RingBuffer {
public:
	RingBuffer();
	void reset();
	bool isFull() {return _nb_data==RBUFFER_SIZE;}
	void add(int16_t);
	int16_t geti(uint16_t);
	int16_t getri(uint16_t);
	int16_t getFirst();
	int16_t getLast();
	int16_t getMinValue();
	int16_t getMaxValue();

	float getRMS32(uint8_t part);

	int16_t* getBufferHandle() {return _data;}

	uint16_t _nb_data;
	uint32_t _tot_data;
private:
	uint16_t _head, _tail;
	int16_t _data[RBUFFER_SIZE];
};

#endif /* LIBRARIES_RBUFFER_RINGBUFFER_H_ */
