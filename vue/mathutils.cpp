/*
 * mathsutils.c
 *
 *  Created on: 5 mars 2017
 *      Author: Vincent
 */

#include <math.h>
#include "mathutils.h"

#define NRF_LOG_MODULE_NAME "MATH"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

double sq(double nb) {
	return pow(nb, 2);
}

// rotation d'angle angle et de centre (cx, cy)
void rotate_point(float angle, uint16_t cx, uint16_t cy,
		uint16_t x1, uint16_t y1, uint16_t &x2, uint16_t &y2) {

	float tmp1, tmp2, tmp3, tmp4;

	float angle_rad = angle * PI / 180.;

	// coordonnees dans (cx, cy)
	tmp1 = x1 - cx;
	tmp2 = y1 - cy;

	// rotation dans (cx, cy)
	tmp3 = tmp1 * cos(angle_rad) - tmp2 * sin(angle_rad);
	tmp4 = tmp1 * sin(angle_rad) + tmp2 * cos(angle_rad);

	// coordonnees dans (0, 0)
	x2 = tmp3 + cx;
	y2 = tmp4 + cy;

	//NRF_LOG_INFO("Nouveaux points: %u %u\r\n", x2, y2);
}


float course_to (float lat1, float long1, float lat2, float long2)
{
  // returns course in degrees (North=0, West=270) from position 1 to position 2,
  // both specified as signed decimal-degrees latitude and longitude.
  // Because Earth is no exact sphere, calculated course may be off by a tiny fraction.
  // Courtesy of Maarten Lamers
  float dlon = long2-long1;
  dlon *= PI / 180.;
  lat1 *= PI / 180.;
  lat2 *= PI / 180.;
  float a1 = sin(dlon) * cos(lat2);
  float a2 = sin(lat1) * cos(lat2) * cos(dlon);
  a2 = cos(lat1) * sin(lat2) - a2;
  a2 = atan2(a1, a2);
  if (a2 < 0.0)
  {
    a2 += TWO_PI;
  }
  a2 *= 180 / PI;
  return a2;
}
