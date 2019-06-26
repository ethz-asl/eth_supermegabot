/*
 * InterpolationTest.cpp
 *
 *  Created on: Jan 28, 2019
 *      Author: Perry Franklin
 */

// gtest
#include <gtest/gtest.h>

// std (for std::pow)
#include <cmath>

// robot utils
#include <robot_utils/interpolators/BasicFifthOrderInterpolation.hpp>
#include <robot_utils/interpolators/LinearInterpolation.hpp>

void generalInterpolationTests(robot_utils::ScalarInterpolation& testInterpolation) {
  double s, dsdt = nan("");

  // Initial and Final values
  testInterpolation.interpolate(0.0, &s, &dsdt);
  EXPECT_NEAR(s, 0.0, 1e-06);
  testInterpolation.interpolate(1.0, &s, &dsdt);
  EXPECT_NEAR(s, 1.0, 1e-06);

  // Integration test for the derivative
  double integral = 0.0;
  const double resolution = 1e04;
  const double step = 1.0 / resolution;
  for (double x = 0.0; x < 1.0; x += step) {
    // Uses trapezoidal integration
    double df1;
    testInterpolation.interpolate(x, &s, &df1);
    double df2;
    testInterpolation.interpolate(x + step, &s, &df2);
    integral += 0.5 * (df1 + df2) * step;
    ASSERT_NEAR(integral, s, step * 1e-03) << " Integration failed, current x = " << x;
  }
}

void linearInterpolationTruth(double t, double* s, double* dsdt) {
  *s = t;
  *dsdt = 1;
}

TEST(Interpolation, LinearInterpolation) {
  using namespace robot_utils;

  LinearInterpolation linearInterpolation;

  for (double t = 0.0; t <= 1.0; t += 0.02) {
    double test_s, test_dsdt;
    double truth_s, truth_dsdt;
    linearInterpolation.interpolate(t, &test_s, &test_dsdt);
    linearInterpolationTruth(t, &truth_s, &truth_dsdt);
    EXPECT_NEAR(test_s, truth_s, 1e-06);
    EXPECT_NEAR(test_dsdt, truth_dsdt, 1e-06);
  }

  generalInterpolationTests(linearInterpolation);
}

void fifthOrderInterpolationTruth(double t, double* s, double* dsdt) {
  *s = 6.0 * std::pow((t), 5) - 15.0 * std::pow((t), 4) + 10.0 * std::pow((t), 3);
  *dsdt = (30.0 * std::pow((t), 4) - 60.0 * std::pow((t), 3) + 30.0 * std::pow((t), 2));
}

TEST(Interpolation, BasicFifthOrderInterpolation) {
  using namespace robot_utils;

  BasicFifthOrderInterpolation basicFifthOrderInterpolation;

  for (double t = 0.0; t <= 1.0; t += 0.02) {
    double test_s, test_dsdt;
    double truth_s, truth_dsdt;
    basicFifthOrderInterpolation.interpolate(t, &test_s, &test_dsdt);
    fifthOrderInterpolationTruth(t, &truth_s, &truth_dsdt);
    EXPECT_NEAR(test_s, truth_s, 1e-06);
    EXPECT_NEAR(test_dsdt, truth_dsdt, 1e-06);
  }

  generalInterpolationTests(basicFifthOrderInterpolation);
}
