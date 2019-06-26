/*
 * CumulativeMovingAverageFilter.hpp
 *
 *  Created on: Feb, 2017
 *      Author: Christian Gehring
 */

#pragma once

namespace robot_utils {
//! Cumulative Moving Average Filter
/*! Assumes that value has method setZero().
 * Used with kindr objects.
 */
template<typename Value_>
class CumulativeMovingAverageFilter {
 public:
  typedef typename Value_::Scalar Scalar;
 public:
  CumulativeMovingAverageFilter() {}
  virtual ~CumulativeMovingAverageFilter() {}

  void reset() {
    numCmaPointsReceived_ = 0u;
    cumulativeMovingAverage_.setZero();
  }

  void update(const Value_& datum) {
    cumulativeMovingAverage_ = (datum + cumulativeMovingAverage_*(Scalar)numCmaPointsReceived_)/((Scalar)numCmaPointsReceived_+(Scalar)1.0);
    numCmaPointsReceived_++;
  }

  const Value_& getValue() const {
    return cumulativeMovingAverage_;
  }

  unsigned int getNumSamples() const {
    return numCmaPointsReceived_;
  }
 protected:
  Value_ cumulativeMovingAverage_;
  unsigned int numCmaPointsReceived_ = 0u;
};

} // namespace
