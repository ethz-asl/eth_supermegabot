/*
 * LinearInterpolation.hpp
 *
 *  Created on: Jan 29, 2019
 *      Author: Perry Franklin
 */

#pragma once

#include <robot_utils/interpolators/ScalarInterpolation.hpp>

namespace robot_utils {

/**
 * Performs Linear Interpolation (ie s = t, dsdt = 1.0)
 */
class LinearInterpolation : public ScalarInterpolation {
 public:
  LinearInterpolation();
  ~LinearInterpolation() override = default;

  /**
   * Interpolates between 0 and 1. Inherited from ScalarInterpolation.
   * @param t : input variable, between 0 and 1 for interpolation
   * @param s : output variable, between 0 and 1 for interpolation
   * @param dsdt : output variable, derivative of s
   */
  void interpolate(double t, double* s, double* dsdt) const override;
};

}  // namespace robot_utils
