/*
 * ContinuousTimeTransferFunctionTest.cpp
 *
 *  Created on: Feb 20, 2018
 *      Author: Philipp Leemann
 */

// gtest
#include <gtest/gtest.h>

// robot utils
#include "robot_utils/filters/ContinuousTimeTransferFunction.hpp"

#include <Eigen/Core>

/*
 * Tests for the transformation of CD to DT transfer function.
 * Test data generated with matlab (see first part of matlab_filter_coeff.m)
 */


TEST(ContinuousTimeTransferFunction, FirstOrderCoefficientTest) {

  constexpr double dt = 0.0025;
  constexpr double gain = 2.0;
  constexpr double tau = 0.01;

  robot_utils::ContinuousTimeTransferFunction<float, 1> filter(dt, {gain, 0.0}, {1.0, tau});

  ASSERT_NEAR(0.222222222222222, filter.getNumeratorCoefficients()[0], 1e-14);
  ASSERT_NEAR(0.222222222222222, filter.getNumeratorCoefficients()[1], 1e-14);
  ASSERT_NEAR(1.0, filter.getDenominatorCoefficients()[0], 1e-14);
  ASSERT_NEAR(-0.777777777777778, filter.getDenominatorCoefficients()[1], 1e-14);
}


TEST(ContinuousTimeTransferFunction, FirstOrderDerivativeCoefficientTest) {

    constexpr double dt = 0.0025;
    constexpr double gain = 2.0;
    constexpr double tau = 0.01;

    robot_utils::ContinuousTimeTransferFunction<float, 1> filter(dt, {0.0, gain}, {1.0, tau});

    ASSERT_NEAR(177.777777777778, filter.getNumeratorCoefficients()[0], 1e-12);
    ASSERT_NEAR(-177.777777777778, filter.getNumeratorCoefficients()[1], 1e-12);
    ASSERT_NEAR(1.0, filter.getDenominatorCoefficients()[0], 1e-12);
    ASSERT_NEAR(-0.777777777777778, filter.getDenominatorCoefficients()[1], 1e-12);
}

TEST(ContinuousTimeTransferFunction, SecondOrderCoefficientTest) {

    constexpr double dt = 0.0025;
    constexpr double gain = 2.0;
    constexpr double tau = 0.01;
    constexpr double damp = 0.9;

    robot_utils::ContinuousTimeTransferFunction<float, 2> filter(dt, {gain, 0.0, 0.0}, {1.0, 2*tau*damp, tau*tau});

    ASSERT_NEAR(0.0251889168765743, filter.getNumeratorCoefficients()[0], 1e-14);
    ASSERT_NEAR(0.0503778337531486, filter.getNumeratorCoefficients()[1], 1e-14);
    ASSERT_NEAR(0.0251889168765743, filter.getNumeratorCoefficients()[2], 1e-14);
    ASSERT_NEAR(1.0, filter.getDenominatorCoefficients()[0], 1e-14);
    ASSERT_NEAR(-1.58690176322418, filter.getDenominatorCoefficients()[1], 1e-14);
    ASSERT_NEAR(0.63727959697733, filter.getDenominatorCoefficients()[2], 1e-14);
}

TEST(ContinuousTimeTransferFunction, SecondOrderDerivativeCoefficientTest) {

    constexpr double dt = 0.0025;
    constexpr double gain = 2.0;
    constexpr double tau = 0.01;
    constexpr double damp = 0.9;

    robot_utils::ContinuousTimeTransferFunction<float, 2> filter(dt, {0.0, gain, 0.0}, {1.0, 2*tau*damp, tau*tau});

    ASSERT_NEAR(20.1511335012594, filter.getNumeratorCoefficients()[0], 1e-12);
    ASSERT_NEAR(0.0, filter.getNumeratorCoefficients()[1], 1e-12);
    ASSERT_NEAR(-20.1511335012594, filter.getNumeratorCoefficients()[2], 1e-12);
    ASSERT_NEAR(1.0, filter.getDenominatorCoefficients()[0], 1e-12);
    ASSERT_NEAR(-1.58690176322418, filter.getDenominatorCoefficients()[1], 1e-12);
    ASSERT_NEAR(0.63727959697733, filter.getDenominatorCoefficients()[2], 1e-12);
}

TEST(ContinuousTimeTransferFunction, FourthOrderNumeratorCoefficientTest) {

    constexpr double dt = 0.0025;
    constexpr double gain = 2.0;
    constexpr double tau = 0.01;
    constexpr double damp = 0.9;

    robot_utils::ContinuousTimeTransferFunction<float, 4> filter(dt, {2.1, 1.4, 1.3, 1.2, 1.1}, {1.0, 0.0, 0.0, 0.0, 0.0});

    ASSERT_NEAR(451175233122.1, filter.getNumeratorCoefficients()[0], 1e-1);
    ASSERT_NEAR(-1803468797751.6, filter.getNumeratorCoefficients()[1], 1e-1);
    ASSERT_NEAR(2703358336012.6, filter.getNumeratorCoefficients()[2], 1e-1);
    ASSERT_NEAR(-1801011202231.6, filter.getNumeratorCoefficients()[3], 1e-1);
    ASSERT_NEAR(449946430882.1, filter.getNumeratorCoefficients()[4], 1e-1);
    ASSERT_NEAR(1.0, filter.getDenominatorCoefficients()[0], 1e-14);
    ASSERT_NEAR(4.0, filter.getDenominatorCoefficients()[1], 1e-14);
    ASSERT_NEAR(6.0, filter.getDenominatorCoefficients()[2], 1e-14);
    ASSERT_NEAR(4.0, filter.getDenominatorCoefficients()[3], 1e-14);
    ASSERT_NEAR(1.0, filter.getDenominatorCoefficients()[4], 1e-14);
}


TEST(ContinuousTimeTransferFunction, FourthOrderDenominatorCoefficientTest) {

    constexpr double dt = 0.0025;
    constexpr double gain = 2.0;
    constexpr double tau = 0.01;
    constexpr double damp = 0.9;

    robot_utils::ContinuousTimeTransferFunction<float, 4> filter(dt, {-1e5, 0.0, 0.0, 0.0, 0.0}, {0.05, 0.04, -0.03, -0.02, 0.01});

    ASSERT_NEAR(-2.4475365450389e-05, filter.getNumeratorCoefficients()[0], 1e-18);
    ASSERT_NEAR(-9.79014618015558e-05, filter.getNumeratorCoefficients()[1], 1e-18);
    ASSERT_NEAR(-0.000146852192702334, filter.getNumeratorCoefficients()[2], 1e-18);
    ASSERT_NEAR(-9.79014618015558e-05, filter.getNumeratorCoefficients()[3], 1e-18);
    ASSERT_NEAR(-2.4475365450389e-05, filter.getNumeratorCoefficients()[4], 1e-18);
    ASSERT_NEAR(1.0, filter.getDenominatorCoefficients()[0], 1e-10);
    ASSERT_NEAR(-4.0050313048343, filter.getDenominatorCoefficients()[1], 1e-14);
    ASSERT_NEAR(6.01507521170135, filter.getDenominatorCoefficients()[2], 1e-14);
    ASSERT_NEAR(-4.01505644585125, filter.getDenominatorCoefficients()[3], 1e-14);
    ASSERT_NEAR(1.00501253918001, filter.getDenominatorCoefficients()[4], 1e-14);
}