/*
 * LinearInterpolation.cpp
 *
 *  Created on: Jan 29, 2019
 *      Author: Perry Franklin
 */

#include <robot_utils/interpolators/LinearInterpolation.hpp>
#include <robot_utils/math/math.hpp>

namespace robot_utils {

LinearInterpolation::LinearInterpolation() : ScalarInterpolation("Linear Interpolation") {}

void LinearInterpolation::interpolate(double t, double* s, double* dsdt) const {
  *s = t;
  *dsdt = 1.0;
}

}  // namespace robot_utils
