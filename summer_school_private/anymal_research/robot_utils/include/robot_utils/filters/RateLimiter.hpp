/*
 * RateLimiter.hpp
 *
 *  Created on: Jan 4, 2019
 *      Author: jelavice
 */

#pragma once

#include <memory>

namespace robot_utils {

class RateLimiter
{

 public:

  RateLimiter() = delete;
  RateLimiter(double dt, double maxRisingSlope = 1.0, double maxFallingSlope = -1.0);
  virtual ~RateLimiter() = default;

  double limitRateOfChange(double input);


  //setters
  void setMaxRisingSlope(double slope)
  {
    maxRisingSlope_ = slope;
  }
  void setMaxFallingSlope(double slope)
  {
    maxFallingSlope_ = slope;
  }
  void setDt(double dt)
  {
    dt_ = dt;
  }

  //getters
  double getMaxRisingSlope() const
  {
    return maxRisingSlope_;
  }
  double getMaxFallingSlope() const
  {
    return maxFallingSlope_;
  }
  double getDt() const
  {
    return dt_;
  }

 private:

  double inputPrev_;
  bool firstCall_;
  double maxRisingSlope_;
  double maxFallingSlope_;
  double dt_;

};

using RateLimiterPtr = std::shared_ptr<RateLimiter>;

} /* namespace */
