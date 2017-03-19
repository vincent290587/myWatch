/*
 * mathutils.h
 *
 *  Created on: 5 mars 2017
 *      Author: Vincent
 */

#ifndef VUE_MATHUTILS_H_
#define VUE_MATHUTILS_H_

#include "Arduino.h"

#define PI 3.14159265
#define TWO_PI (2.*PI)

#define CLIP(X,Y,Z) (MIN(MAX(X,Y),Z))

double sq(double nb);

void rotate_point(float angle, uint16_t cx, uint16_t cy,
		uint16_t x1, uint16_t y1, uint16_t &x2, uint16_t &y2);

float course_to (float lat1, float long1, float lat2, float long2);


#endif /* VUE_MATHUTILS_H_ */
