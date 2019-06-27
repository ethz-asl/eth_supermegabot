/*
 * BasicFifthOrderInterpolation.hpp
 *
 *  Created on: Jan 28, 2019
 *      Author: Perry Franklin
 */

#pragma once

#include <robot_utils/interpolators/ScalarInterpolation.hpp>

namespace robot_utils {

/**
 * Provides interpolation using a fifth order polynomial.
 */
class BasicFifthOrderInterpolation : public ScalarInterpolation {
 public:
  BasicFifthOrderInterpolation();
  ~BasicFifthOrderInterpolation() override = default;

  /**
   * Interpolates between 0 and 1. Inherited from ScalarInterpolation.
   * @param t : input variable, between 0 and 1 for interpolation
   * @param s : output variable, between 0 and 1 for interpolation
   * @param dsdt : output variable, derivative of s
   */
  void interpolate(double t, double* s, double* dsdt) const override;
};

}  // namespace robot_utils
