/*
 * BasicFifthOrderInterpolation.cpp
 *
 *  Created on: Jan 28, 2019
 *      Author: Koen Kraemer, Perry Franklin
 */

#include <robot_utils/interpolators/BasicFifthOrderInterpolation.hpp>
#include <robot_utils/math/math.hpp>

namespace robot_utils {

BasicFifthOrderInterpolation::BasicFifthOrderInterpolation() : ScalarInterpolation("Basic Fifth Order Interpolation") {}

void BasicFifthOrderInterpolation::interpolate(double t, double* s, double* dsdt) const {
  *s = 6.0 * fastPow((t), 5) - 15.0 * fastPow((t), 4) + 10.0 * fastPow((t), 3);
  *dsdt = (30.0 * fastPow((t), 4) - 60.0 * fastPow((t), 3) + 30.0 * fastPow((t), 2));
}

}  // namespace robot_utils
