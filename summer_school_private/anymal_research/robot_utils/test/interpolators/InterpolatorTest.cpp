/*
 * InterpolatorTest.cpp
 *
 *  Created on: Jan 28, 2019
 *      Author: Perry Franklin
 */

// gtest
#include <gtest/gtest.h>

// kindr gtest extension
#include <kindr/common/gtest_eigen.hpp>

// Eigen
#include <Eigen/Core>

// robot utils
#include <robot_utils/interpolators/BasicFifthOrderInterpolation.hpp>
#include <robot_utils/interpolators/Interpolator.hpp>
#include <robot_utils/interpolators/LinearInterpolation.hpp>
#include <robot_utils/math/math.hpp>

TEST(Interpolator, InterpolatorDouble) {
  using namespace robot_utils;

  using Interpolation = robot_utils::LinearInterpolation;
  using ValueType = double;

  Interpolator<ValueType, Interpolation> interpolator;

  Interpolation groundTruthInterpolation;

  ValueType initial = 4.5;
  ValueType final = -2.0;
  ValueType difference = final - initial;

  interpolator.initialize(initial, final, 2.0);

  const double duration = 2.0;
  const double timestep = 0.4;

  for (double time = 0.0; time <= duration + timestep; /*advance at the bottom of for loop*/) {
    // Interpolators are stationary before 0.0 and after duration
    double t = mapTo01Range(time, 0.0, duration);
    double s, dsdt;
    groundTruthInterpolation.interpolate(t, &s, &dsdt);
    double sDot = 0.0;
    if (time >= 0.0 && time < duration) {
      sDot = dsdt / duration;
    }

    ValueType currentValueTruth = (1 - s) * initial + (s)*final;

    EXPECT_EQ(interpolator.getProgress(), t) << "Time = " << time << "/" << duration;
    EXPECT_EQ(time >= duration, interpolator.isFinished()) << "Time = " << time << "/" << duration;

    EXPECT_NEAR(currentValueTruth, interpolator.getCurrentValue(), 1.0e-9) << "Time = " << time << "/" << duration;

    ValueType derivativeTruth = sDot * difference;

    EXPECT_NEAR(derivativeTruth, interpolator.getCurrentDerivative(), 1.0e-9) << "Time = " << time << "/" << duration;

    // Advance the time
    time += timestep;
    interpolator.advance(timestep);
  }
}

TEST(Interpolator, InterpolatorUnevenTimestep) {
  using namespace robot_utils;

  using Interpolation = robot_utils::LinearInterpolation;
  using ValueType = double;

  Interpolator<ValueType, Interpolation> interpolator;

  Interpolation groundTruthInterpolation;

  ValueType initial = 4.5;
  ValueType final = -2.0;
  ValueType difference = final - initial;

  interpolator.initialize(initial, final, 2.0);

  const double duration = 2.0;
  const double timestep = 0.35;

  for (double time = 0.0; time <= duration + timestep; /*advance at the bottom of for loop*/) {
    // Interpolators are stationary before 0.0 and after duration
    double t = mapTo01Range(time, 0.0, duration);
    double s, dsdt;
    groundTruthInterpolation.interpolate(t, &s, &dsdt);
    double sDot = 0.0;
    if (time >= 0.0 && time < duration) {
      sDot = dsdt / duration;
    }

    ValueType currentValueTruth = (1 - s) * initial + (s)*final;

    EXPECT_EQ(interpolator.getProgress(), t) << "Time = " << time << "/" << duration;
    EXPECT_EQ(time >= duration, interpolator.isFinished()) << "Time = " << time << "/" << duration;

    EXPECT_NEAR(currentValueTruth, interpolator.getCurrentValue(), 1.0e-9) << "Time = " << time << "/" << duration;

    ValueType derivativeTruth = sDot * difference;

    EXPECT_NEAR(derivativeTruth, interpolator.getCurrentDerivative(), 1.0e-9) << "Time = " << time << "/" << duration;

    // Advance the time
    time += timestep;
    interpolator.advance(timestep);
  }
}

TEST(Interpolator, InterpolatorEigenMatrix) {
  using namespace robot_utils;

  using ValueType = Eigen::Matrix3d;

  using Interpolation = robot_utils::LinearInterpolation;

  Interpolator<ValueType, Interpolation> interpolator;

  Interpolation groundTruthInterpolation;

  ValueType initial = Eigen::Matrix3d::Identity();
  ValueType final = 2.0 * Eigen::Matrix3d::Identity();
  ValueType difference = final - initial;

  interpolator.initialize(initial, final, 2.0);

  const double duration = 2.0;
  const double timestep = 0.4;

  for (double time = 0.0; time <= duration + timestep; /*advance at the bottom of for loop*/) {
    // Interpolators are stationary before 0.0 and after duration
    double t = mapTo01Range(time, 0.0, duration);
    double s, dsdt;
    groundTruthInterpolation.interpolate(t, &s, &dsdt);
    double sDot = 0.0;
    if (time >= 0.0 && time < duration) {
      sDot = dsdt / duration;
    }

    ValueType currentValueTruth = (1 - s) * initial + (s)*final;

    EXPECT_EQ(interpolator.getProgress(), t) << "Time = " << time << "/" << duration;
    EXPECT_EQ(time >= duration, interpolator.isFinished()) << "Time = " << time << "/" << duration;

    KINDR_ASSERT_DOUBLE_MX_EQ(currentValueTruth, interpolator.getCurrentValue(), 1.0e-9, "Time = " << time << "/" << duration);

    ValueType derivativeTruth = sDot * difference;

    KINDR_ASSERT_DOUBLE_MX_EQ(derivativeTruth, interpolator.getCurrentDerivative(), 1.0e-9, "Time = " << time << "/" << duration);

    // Advance the time
    time += timestep;
    interpolator.advance(timestep);
  }
}

TEST(Interpolator, InterpolatorEigenFifthOrder) {
  using namespace robot_utils;

  using Interpolation = robot_utils::BasicFifthOrderInterpolation;
  using ValueType = double;

  Interpolator<ValueType, Interpolation> interpolator;

  Interpolation groundTruthInterpolation;

  ValueType initial = 4.5;
  ValueType final = -2.0;
  ValueType difference = final - initial;

  interpolator.initialize(initial, final, 2.0);

  const double duration = 2.0;
  const double timestep = 0.35;

  for (double time = 0.0; time <= duration + timestep; /*advance at the bottom of for loop*/) {
    // Interpolators are stationary before 0.0 and after duration
    double t = mapTo01Range(time, 0.0, duration);
    double s, dsdt;
    groundTruthInterpolation.interpolate(t, &s, &dsdt);
    double sDot = 0.0;
    if (time >= 0.0 && time < duration) {
      sDot = dsdt / duration;
    }

    ValueType currentValueTruth = (1 - s) * initial + (s)*final;

    EXPECT_EQ(interpolator.getProgress(), t) << "Time = " << time << "/" << duration;
    EXPECT_EQ(time >= duration, interpolator.isFinished()) << "Time = " << time << "/" << duration;

    EXPECT_NEAR(currentValueTruth, interpolator.getCurrentValue(), 1.0e-9) << "Time = " << time << "/" << duration;

    ValueType derivativeTruth = sDot * difference;

    EXPECT_NEAR(derivativeTruth, interpolator.getCurrentDerivative(), 1.0e-9) << "Time = " << time << "/" << duration;

    // Advance the time
    time += timestep;
    interpolator.advance(timestep);
  }
}

TEST(Interpolator, PoseInterpolator) {
  using namespace robot_utils;

  using Interpolation = robot_utils::LinearInterpolation;
  using ValueType = kindr::HomTransformQuatD;
  using Rotation = ValueType::Rotation;
  using DerivativeType = typename DerivativeTraits<ValueType>::type;

  Interpolator<ValueType, Interpolation> poseInterpolator;

  Interpolation groundTruthInterpolation;

  ValueType initial({0, 3, -4}, {0.707, 0, 0.0, 0.707});
  initial.getRotation().fix();
  ValueType final({0.1, -0.8, 10.0}, {0.5, 0.5, 0.5, 0.5});
  final.getRotation().fix();
  ValueType::Position translationDifference_ = final.getPosition() - initial.getPosition();
  Eigen::Vector3d rotationDifference = final.getRotation().boxMinus(initial.getRotation());

  poseInterpolator.initialize(initial, final, 2.0);

  const double duration = 2.0;
  const double timestep = 0.35;

  for (double time = 0.0; time <= duration + timestep; /*advance at the bottom of for loop*/) {
    // Interpolators are stationary before 0.0 and after duration
    double t = mapTo01Range(time, 0.0, duration);
    double s, dsdt;
    groundTruthInterpolation.interpolate(t, &s, &dsdt);
    double sDot = 0.0;
    if (time >= 0.0 && time < duration) {
      sDot = dsdt / duration;
    }

    ValueType currentTruth;
    currentTruth.getPosition() = (1 - s) * initial.getPosition() + s * (final.getPosition());
    currentTruth.getRotation() = initial.getRotation().boxPlus(s * rotationDifference);

    EXPECT_EQ(poseInterpolator.getProgress(), t) << "Time = " << time << "/" << duration;
    EXPECT_EQ(time >= duration, poseInterpolator.isFinished()) << "Time = " << time << "/" << duration;

    KINDR_ASSERT_DOUBLE_MX_EQ(currentTruth.getPosition().toImplementation(),
                              poseInterpolator.getCurrentValue().getPosition().toImplementation(), 1.0e-9,
                              "Time = " << time << "/" << duration);
    EXPECT_TRUE(currentTruth.getRotation().isNear(poseInterpolator.getCurrentValue().getRotation(), 1.0e-9))
        << "Time = " << time << "/" << duration;

    DerivativeType currentTwistTruth;
    currentTwistTruth.getTranslationalVelocity() = DerivativeType::PositionDiff(sDot * translationDifference_);
    currentTwistTruth.getRotationalVelocity() = DerivativeType::RotationDiff(sDot * rotationDifference);

    KINDR_ASSERT_DOUBLE_MX_EQ(currentTwistTruth.getVector(), poseInterpolator.getCurrentDerivative().getVector(), 1.0e-9,
                              "Time = " << time << "/" << duration);
    //    KINDR_ASSERT_DOUBLE_MX_EQ(currentTwistTruth.getVector(), poseInterpolator.getCurrentDerivative().getVector(), 1.0e-9,
    //                              "Time = " << time << "/" << duration);

    //     Advance the time
    time += timestep;
    poseInterpolator.advance(timestep);
  }
}
