/*
 * HRFilter.cpp
 *
 *  Created on: 2 avr. 2017
 *      Author: Vincent
 */

#include "math.h"
#include "HRFilter.h"

HRFilter::HRFilter() {
	resetFilter();
}

void HRFilter::resetFilter() {
	_learn = 0;
	vx1 = 0; vx2 = 0; vx3 = 0; vx4 = 0; vx5 = 0; vx6 = 0; vx7 = 0;
	vy1 = 0; vy2 = 0; vy3 = 0; vy4 = 0; vy5 = 0; vy6 = 0; vy7 = 0;
}


float HRFilter::filter_cheb1(int32_t new_data) {

  float coarse = filter_coarse(new_data);
  float res = coarse;

  vx7 = vx6; vx6 = vx5; vx5 = vx4; vx4 = vx3; vx3 = vx2; vx2 = vx1; vx1 = new_data;
  vy7 = vy6; vy6 = vy5; vy5 = vy4; vy4 = vy3; vy3 = vy2; vy2 = vy1;

  res = vx1 * cx1 + vx2 * cx2 + vx3 * cx3 + vx4 * cx4 + vx5 * cx5 + vx6 * cx6 + vx7 * cx7;
  res -= vy2 * cy2 + vy3 * cy3 + vy4 * cy4 + vy5 * cy5 + vy6 * cy6 + vy7 * cy7;

  if (_learn < 75) {
	  vy1 = coarse;
	  res = coarse;
	  _learn++;
  } else if (abs(coarse - res) > 5000) {
	  vy1 = coarse;
	  res = coarse;
	  _learn = 0;
  } else {
	  vy1 = res;
  }

  return res;
}



float HRFilter::filter_coarse(int32_t new_data) {

  // working coarse filter !
  const float cx1_ = -0.0434251;
  const float cx2_ = 0.4266939;
  const float cx3_ = -0.3832688;

  const float cy2_ = -1.5013042;
  const float cy3_ = 0.6601563;

  static float vx1_ = 0, vx2_ = 0, vx3_ = 0;
  static float vy1_ = 0, vy2_ = 0, vy3_ = 0;

  float res;

  vx3_ = vx2_; vx2_ = vx1_; vx1_ = new_data;
  vy3_ = vy2_; vy2_ = vy1_;

  res = vx1_ * cx1_ + vx2_ * cx2_ + vx3_ * cx3_;
  res -= vy2_ * cy2_ + vy3_ * cy3_;

  vy1_ = res;

  return res;
}
