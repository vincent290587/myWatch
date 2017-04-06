/*
 * HRFilter.h
 *
 *  Created on: 2 avr. 2017
 *      Author: Vincent
 */

#ifndef LIBRARIES_MAX30102_HRFILTER_H_
#define LIBRARIES_MAX30102_HRFILTER_H_

#include "Arduino.h"

class HRFilter {
public:
	HRFilter();
	void resetFilter();
	float filter_cheb1(int32_t new_data);


private:
	uint16_t _learn;
	const float cx1 = 0.0116512;
	const float cx2 = 0;
	const float cx3 = - 0.0349536;
	const float cx4 = 0;
	const float cx5 = 0.0349536;
	const float cx6 = 0;
	const float cx7 = -0.0116512;

	const float cy2 = - 4.8088874;
	const float cy3 = 9.9059709;
	const float cy4 = - 11.233593;
	const float cy5 = 7.4148886;
	const float cy6 = - 2.7021079;
	const float cy7 = 0.4242391;

	float vx1 = 0, vx2 = 0, vx3 = 0, vx4 = 0, vx5 = 0, vx6 = 0, vx7 = 0;
	float vy1 = 0, vy2 = 0, vy3 = 0, vy4 = 0, vy5 = 0, vy6 = 0, vy7 = 0;

	float filter_coarse(int32_t new_data);
};

#endif /* LIBRARIES_MAX30102_HRFILTER_H_ */
