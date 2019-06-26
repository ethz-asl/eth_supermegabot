/*
 * Interpolator.hpp
 *
 *  Created on: Jan 28, 2019
 *      Author: Perry Franklin
 */

#pragma once

#include <memory>

#include <robot_utils/interpolators/ScalarInterpolation.hpp>
#include <robot_utils/math/DerivativeTraits.hpp>

namespace robot_utils {

/**
 * Interpolator for any linear mathematical type.
 *   ValueType must have operator+ and operator* defined (as addition and multiplication, respectively)
 *             Users should also verify that DerivativeTraits, DifferenceTraits, and InterpolationTraits are valid for ValueType
 *   InterpolationType must inherit from ScalarInterpolation
 */
template <typename ValueType_, typename InterpolationType_>
class Interpolator {

  using Interpolation = InterpolationType_;
  static_assert(std::is_base_of<ScalarInterpolation, Interpolation>::value, 
                  "You tried to instantiate a constructor with an InterpolationType_ that is not a derived class of ScalarInterpolation");

 public:
  using ValueType = ValueType_;
  using DerivativeType = typename robot_utils::DerivativeTraits<ValueType>::type;

 public:
  /**
   * Constructor
   * @param interpolation: unique ptr to a interpolation method (will be set null)
   */
  Interpolator();
  virtual ~Interpolator() = default;

  /**
   * Initializes the interpolator
   * @param initialValue: initial (starting) value for interpolation
   * @param finalValue: final value for interpolation
   * @param motionDuration: the total duration of the motion
   * @return success
   * @note sets the internal time to 0 (after initialize, the interpolator always starts at initalValue)
   */
  bool initialize(const ValueType& initialValue, const ValueType& finalValue, double motionDuration);

  /**
   * Advances the interpolator by duration dt, then updates internal values
   * @param dt: delta time
   * @return success
   */
  bool advance(double dt);

  /**
   * Gets the current value of the interpolation
   * @return current value
   */
  const ValueType& getCurrentValue() const;

  /**
   * Gets the current derivative of the interpolation
   * @return current derivative
   */
  const DerivativeType& getCurrentDerivative() const;

  /**
   * Checks if the interpolation is finished (if time is past motionDuration)
   * @return finished
   */
  bool isFinished() const;

  /**
   * Aborts the interpolation (ceases motion, sets derivative to 0.0)
   * @return success
   */
  bool abort();

  /**
   * Gets the progress of the interpolation, as a percentage of total time
   * @return percentage complete (percentage time passed) (in [0,1])
   */
  double getProgress() const;

 public:
  void updateCurrentValues();

  Interpolation interpolation_;

  ValueType initialValue_;
  ValueType finalValue_;
  ValueType currentValue_;
  DerivativeType currentDerivative_;

  double motionDuration_;
  double time_;
  double timeFraction_;  // value between 0 and 1 indicating point in interpolation between initialPose_ and finalPose_

  DerivativeType totalDifference_;

  bool interpolationIsFinished_;
};

}  // namespace robot_utils

#include <robot_utils/interpolators/Interpolator.tpp>
