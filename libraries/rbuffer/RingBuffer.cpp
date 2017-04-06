/*
 * RingBuffer.cpp
 *
 *  Created on: 2 avr. 2017
 *      Author: Vincent
 */

#include "math.h"
#include "RingBuffer.h"

RingBuffer::RingBuffer() {

	reset();

}

void RingBuffer::reset() {
	_nb_data = 0;
	_head = 0;
	_tail = 0;
	_tot_data = 0;
}

void RingBuffer::add(int16_t new_data) {

	if (_nb_data < RBUFFER_SIZE) {
		_data[_tail] = new_data;

		_tail++;
		_nb_data++;
	} else {
		_data[_tail % RBUFFER_SIZE] = new_data;

		_head++;
		_head = _head % RBUFFER_SIZE;

		_tail++;
		_tail = _tail % RBUFFER_SIZE;
	}

	_tot_data++;

}

int16_t RingBuffer::geti(uint16_t i) {

	uint16_t ind = _head + i;

	return _data[ind % RBUFFER_SIZE];
}

int16_t RingBuffer::getri(uint16_t i) {

	uint16_t ind = _tail + RBUFFER_SIZE - i;

	return _data[ind % RBUFFER_SIZE];
}

int16_t RingBuffer::getFirst() {

	return _data[_head];

}

int16_t RingBuffer::getLast() {

	return _data[_tail];

}

int16_t RingBuffer::getMinValue() {

	int16_t i, res;

	res = _data[0];

	for (i=1; i<_nb_data; i++) {
		if (res > _data[i]) res = _data[i];
	}

	return res;

}

int16_t RingBuffer::getMaxValue() {

	int16_t i, res;

	res = _data[0];

	for (i=1; i<_nb_data; i++) {
		if (res < _data[i]) res = _data[i];
	}

	return res;

}

float RingBuffer::getRMS32(uint8_t part) {

	uint16_t i, ind;
	float res;
	uint32_t tmp=0;

	for (i=0; i<_nb_data; i++) {
		ind = (_head + (part-1)*RBUFFER_SIZE/4 + i) % RBUFFER_SIZE;
		tmp+= _data[ind % RBUFFER_SIZE]*_data[ind % RBUFFER_SIZE];
	}
	res = float(tmp);

	res /= RBUFFER_SIZE/4;
	res = sqrt(res);

	return res;

}
