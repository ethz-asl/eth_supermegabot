/*
 * Interpolator.tpp
 *
 *  Created on: Jan 28, 2019
 *      Author: Perry Franklin
 */

#include <robot_utils/interpolators/Interpolator.hpp>
#include <robot_utils/math/math.hpp>
#include <robot_utils/math/DifferenceTraits.hpp>
#include <robot_utils/math/InterpolationTraits.hpp>
#include <robot_utils/math/MultiplicationTraits.hpp>

#include <cassert>

namespace robot_utils {

template <typename ValueType_, typename InterpolationType_>
Interpolator<ValueType_, InterpolationType_>::Interpolator()
    : initialValue_(),
      finalValue_(),
      interpolationIsFinished_(true),
      motionDuration_(10.0),
      time_(0.0),
      timeFraction_(0.0),
      totalDifference_() {
}

template <typename ValueType_, typename InterpolationType_>
bool Interpolator<ValueType_, InterpolationType_>::initialize(const ValueType_& initialValue, const ValueType_& finalValue, double motionDuration) {
  if (!isFinished()) {
    return false;
  }

  initialValue_ = initialValue;
  finalValue_ = finalValue;
  motionDuration_ = motionDuration;
  interpolationIsFinished_ = false;
  time_ = 0.0;
  timeFraction_ = 0.0;

  totalDifference_= DifferenceTraits<ValueType>::subtract(finalValue_, initialValue_);

  updateCurrentValues();

  return true;
}

template <typename ValueType_, typename InterpolationType_>
bool Interpolator<ValueType_, InterpolationType_>::advance(double dt) {
  if (isFinished()) {
    return true;
  }

  time_ += dt;

  updateCurrentValues();

  return true;
}

template <typename ValueType_, typename InterpolationType_>
bool Interpolator<ValueType_, InterpolationType_>::abort() {
  interpolationIsFinished_ = true;
  currentDerivative_.setZero();
  return true;
}

template <typename ValueType_, typename InterpolationType_>
bool Interpolator<ValueType_, InterpolationType_>::isFinished() const {
  return interpolationIsFinished_;
}

template <typename ValueType_, typename InterpolationType_>
const ValueType_& Interpolator<ValueType_, InterpolationType_>::getCurrentValue() const {
  return currentValue_;
}

template <typename ValueType_, typename InterpolationType_>
const typename Interpolator<ValueType_, InterpolationType_>::DerivativeType& Interpolator<ValueType_, InterpolationType_>::getCurrentDerivative() const {
  return currentDerivative_;
}

template <typename ValueType_, typename InterpolationType_>
double Interpolator<ValueType_, InterpolationType_>::getProgress() const {
  return timeFraction_;
}

template <typename ValueType_, typename InterpolationType_>
void Interpolator<ValueType_, InterpolationType_>::updateCurrentValues() {

  timeFraction_ = mapTo01Range(time_, 0.0, motionDuration_);
  double s = 0.0;
  double dsdTimeFraction = 0.0;
  double sDot = 0.0;

  if (timeFraction_ < 1.0 && timeFraction_ >= 0.0) {
    interpolation_.interpolate(timeFraction_, &s, &dsdTimeFraction);
    sDot = dsdTimeFraction / motionDuration_;
    interpolationIsFinished_ = false;
  } else if (timeFraction_ >= 1.0) {
    s = 1.0;
    dsdTimeFraction = 0.0;
    sDot = 0.0;
    interpolationIsFinished_ = true;
  } else {  // timeFraction < 0.0
    s = 0.0;
    dsdTimeFraction = 0.0;
    sDot = 0.0;
    interpolationIsFinished_ = false;
  }

  currentValue_ = InterpolationTraits<ValueType>::applyInterpolation(s, initialValue_, totalDifference_);
  currentDerivative_ = MultiplicationTraits<DerivativeType>::scalarMultiplication(sDot, totalDifference_);

}

}  // namespace robot_utils
