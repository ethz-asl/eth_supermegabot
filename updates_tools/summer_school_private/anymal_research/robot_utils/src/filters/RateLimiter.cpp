/*
 * RateLimiter.cpp
 *
 *  Created on: Jan 4, 2019
 *      Author: jelavice
 */

#include "robot_utils/filters/RateLimiter.hpp"

namespace robot_utils {

RateLimiter::RateLimiter(double dt, double maxRisingSlope, double maxFallingSlope)
    : inputPrev_(0.0),
      firstCall_(true),
      maxRisingSlope_(maxRisingSlope),
      maxFallingSlope_(maxFallingSlope),
      dt_(dt)
{

}

double RateLimiter::limitRateOfChange(double input)
{

  if (firstCall_) {
    inputPrev_ = input;
    firstCall_ = false;
    return input;
  }

  double returnValue;
  if (input > inputPrev_ + dt_ * maxRisingSlope_) {
    returnValue = inputPrev_ + dt_ * maxRisingSlope_;
  } else if (input < inputPrev_ + dt_ * maxFallingSlope_) {
    returnValue = inputPrev_ + dt_ * maxFallingSlope_;
  } else {
    returnValue = input;
  }

  inputPrev_ = returnValue;

  return returnValue;

}

} /* namesapce */
