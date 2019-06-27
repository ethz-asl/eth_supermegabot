/*
 * Copyright 2018 Timothy Sandy
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <Eigen/Core>
#include <gtest/gtest.h>
#include "confusion/LocalParameterization.h"

using namespace confusion;
const double tolerance = 5e-8;

void check_quats(Eigen::Quaterniond q1, Eigen::Quaterniond q2, Eigen::Vector4d dq_est, bool print = false) {
  //Print results
  Eigen::Quaterniond q2_est(q1.w() + dq_est(0), q1.x() + dq_est(1), q1.y() + dq_est(2), q1.z() + dq_est(3));

  if (print) {
    std::cout << "q1 =[" << q1.w() << "," << q1.x() << "," << q1.y() << "," << q1.z() << "]" << std::endl;
    std::cout << "q2 =[" << q2.w() << "," << q2.x() << "," << q2.y() << "," << q2.z() << "]" << std::endl;
    std::cout << "q2e=[" << q2_est.w() << "," << q2_est.x() << "," << q2_est.y() << "," << q2_est.z() << "]"
              << std::endl;
  }

  Eigen::Vector4d dq;
  dq << q2.w() - q1.w(), q2.x() - q1.x(), q2.y() - q1.y(), q2.z() - q1.z();
//	std::cout << "test: " << dq(0)-dq_est(0) << "," << dq(1)-dq_est(1) << "," <<
//			dq(2)-dq_est(2) << "," << dq(3)-dq_est(3) << std::endl;
  EXPECT_LT(fabs(dq(0) - dq_est(0)), tolerance);
  EXPECT_LT(fabs(dq(1) - dq_est(1)), tolerance);
  EXPECT_LT(fabs(dq(2) - dq_est(2)), tolerance);
  EXPECT_LT(fabs(dq(3) - dq_est(3)), tolerance);
}

//todo Can this be made general, just parameterized by the global and local parameter sizes?
TEST(quatParamPlusTest, ShouldPass) {
  for (int i = 0; i < 100; ++i) {
    QuatParam eqp;
    Eigen::Quaterniond q0(Eigen::Vector4d::Random());
    q0.normalize();

    Eigen::Vector3d dq(Eigen::Vector3d::Random());
    dq *= 1e-5;

    Eigen::Quaterniond q1;
    EXPECT_TRUE(eqp.Plus(q0.coeffs().data(), dq.data(), q1.coeffs().data()));

    Eigen::Matrix<double, 4, 3, Eigen::RowMajor> dq1_ddq;
    EXPECT_TRUE(eqp.ComputeJacobian(q0.coeffs().data(), dq1_ddq.data()));

    //Perturb dq
    Eigen::Vector3d ddq(Eigen::Vector3d::Random());
    ddq *= 1e-5;

    Eigen::Vector3d dqp = dq + ddq;

    Eigen::Quaterniond q1p;
    EXPECT_TRUE(eqp.Plus(q0.coeffs().data(), dqp.data(), q1p.coeffs().data()));

//		std::cout << "q0: " << q0.coeffs().transpose() << std::endl;
//		std::cout << "q1: " << q1.coeffs().transpose() << std::endl;
//		std::cout << "q1p: " << q1p.coeffs().transpose() << std::endl;

    Eigen::Vector4d dq1est_ = dq1_ddq * ddq;
    Eigen::Vector4d dq1est;
    dq1est
        << dq1est_(3), dq1est_(0), dq1est_(1), dq1est_(2); //Need to reorder because of the eigen underlying structure
//		std::cout << "dq1est: " << dq1est.transpose() << std::endl;

    check_quats(q1, q1p, dq1est, false);
  }
}

TEST(quatParamMinusTest, ShouldPass) {
  for (int i = 0; i < 100; ++i) {
    QuatParam eqp;
    Eigen::Quaterniond q0(Eigen::Vector4d::Random());
    q0.normalize();

//		Eigen::Vector3d dq(Eigen::Vector3d::Random());
//		dq *= 1e-4;

    Eigen::Quaterniond q1(Eigen::Vector4d::Random()); //= confusion::quaternionBoxPlus(q0, dq);
    q1.normalize();
//std::cout << "q0: " << q0.coeffs().transpose() << "\nq1: " << q1.coeffs().transpose() << std::endl;

    Eigen::Vector3d dq_calc;
    eqp.boxMinus(q0.coeffs().data(), q1.coeffs().data(), dq_calc.data());
//std::cout << "dq: " << dq.transpose() << "\ndq_calc: " << dq_calc.transpose() << std::endl;

    //Test right derivative
    Eigen::Matrix<double, 3, 4> ddq_dq1 = eqp.boxMinusJacobianRight(q0.coeffs().data(), q1.coeffs().data());
//std::cout << "ddq_dq1:\n" << ddq_dq1 << std::endl;

    //Perturb q1
    Eigen::Vector4d dq1(Eigen::Vector4d::Random());
    dq1 *= 1e-4;

    Eigen::Quaterniond q1p = q1;
    q1p.coeffs() += dq1;
//std::cout << "q1p: " << q1p.coeffs().transpose() << std::endl;
    Eigen::Vector3d dq_calc_p;
    eqp.boxMinus(q0.coeffs().data(), q1p.coeffs().data(), dq_calc_p.data());

//    std::cout << "q0: " << q0.coeffs().transpose() << std::endl;
//    std::cout << "q1: " << q1.coeffs().transpose() << std::endl;
//    std::cout << "q1p: " << q1p.coeffs().transpose() << std::endl;
    Eigen::Vector3d ddq = dq_calc_p - dq_calc;
    Eigen::Vector3d ddq_est = ddq_dq1 * dq1;

//    std::cout << "ddq: " << ddq.transpose() << "\nddq_est: " << ddq_est.transpose() << "\n" << std::endl;

    for (int i = 0; i < 3; ++i)
      EXPECT_LT(fabs(ddq(i) - ddq_est(i)), tolerance);


    //Test left derivative
    Eigen::Matrix<double, 3, 4> ddq_dq0 = eqp.boxMinusJacobianLeft(q0.coeffs().data(), q1.coeffs().data());
//std::cout << "ddq_dq1:\n" << ddq_dq1 << std::endl;

    //Perturb q0
    Eigen::Vector4d dq0(Eigen::Vector4d::Random());
    dq0 *= 1e-4;

    Eigen::Quaterniond q0p = q0;
    q0p.coeffs() += dq0;
//std::cout << "q0p: " << q0p.coeffs().transpose() << std::endl;
    Eigen::Vector3d dq0_calc_p;
    eqp.boxMinus(q0p.coeffs().data(), q1.coeffs().data(), dq0_calc_p.data());

//    std::cout << "q1: " << q1.coeffs().transpose() << std::endl;
//    std::cout << "q0: " << q0.coeffs().transpose() << std::endl;
//    std::cout << "q0p: " << q0p.coeffs().transpose() << std::endl;
    Eigen::Vector3d ddq0 = dq0_calc_p - dq_calc;
    Eigen::Vector3d ddq0_est = ddq_dq0 * dq0;

//    std::cout << "ddq: " << ddq0.transpose() << "\nddq_est: " << ddq0_est.transpose() << "\n" << std::endl;

    for (int i = 0; i < 3; ++i)
      EXPECT_LT(fabs(ddq0(i) - ddq0_est(i)), tolerance);

  }
}

TEST(quatPlusMinusTest, ShouldPass) {
  for (int i = 0; i < 10; ++i) {
    QuatParam eqp;
    Eigen::Quaterniond q0(Eigen::Vector4d::Random());
    q0.normalize();

    Eigen::Vector3d dq(Eigen::Vector3d::Random());
    dq *= 1e-3;

    Eigen::Quaterniond q1;
    EXPECT_TRUE(eqp.Plus(q0.coeffs().data(), dq.data(), q1.coeffs().data()));
//		std::cout << "q0: " << q0.coeffs().transpose() << "\nq1: " << q1.coeffs().transpose() << std::endl;

    Eigen::Vector3d dq_calc;
    eqp.boxMinus(q0.coeffs().data(), q1.coeffs().data(), dq_calc.data());

//		std::cout << "dq : " << dq.transpose() << "\ndqc: " << dq_calc.transpose() << "\n\n" << std::endl;

//		for (int i=0; i<3; ++i)
//			EXPECT_LT(fabs(ddq(0)-ddq_est(0)),tolerance);
  }
}

TEST(fixedYawParamPlusTest, ShouldPass) {
  for (int i = 0; i < 100; ++i) {
    FixedYawParameterization fyp;
    Eigen::Quaterniond q0(Eigen::Vector4d::Random());
    q0.normalize();

    Eigen::Vector2d dq(Eigen::Vector2d::Random());
    dq *= 1e-5;

    Eigen::Quaterniond q1;
    EXPECT_TRUE(fyp.Plus(q0.coeffs().data(), dq.data(), q1.coeffs().data()));

    Eigen::Matrix<double, 4, 2, Eigen::RowMajor> dq1_ddq;
    EXPECT_TRUE(fyp.ComputeJacobian(q0.coeffs().data(), dq1_ddq.data()));

    //Perturb dq
    Eigen::Vector2d ddq(Eigen::Vector2d::Random());
    ddq *= 1e-5;

    Eigen::Vector2d dqp = dq + ddq;

    Eigen::Quaterniond q1p;
    EXPECT_TRUE(fyp.Plus(q0.coeffs().data(), dqp.data(), q1p.coeffs().data()));

//		std::cout << "q0: " << q0.coeffs().transpose() << std::endl;
//		std::cout << "q1: " << q1.coeffs().transpose() << std::endl;
//		std::cout << "q1p: " << q1p.coeffs().transpose() << std::endl;

    Eigen::Vector4d dq1est_ = dq1_ddq * ddq;
    Eigen::Vector4d dq1est;
    dq1est
        << dq1est_(3), dq1est_(0), dq1est_(1), dq1est_(2); //Need to reorder because of the eigen underlying structure
//		std::cout << "dq1est: " << dq1est.transpose() << std::endl;

    check_quats(q1, q1p, dq1est, false);
  }
}

TEST(fixedYawParamMinusTest, ShouldPass) {
  for (int i = 0; i < 100; ++i) {
    FixedYawParameterization fyp;
    Eigen::Quaterniond q0(Eigen::Vector4d::Random());
    q0.normalize();

    Eigen::Quaterniond q1(Eigen::Vector4d::Random());
    q1.normalize();

    Eigen::Vector2d dq_calc;
    fyp.boxMinus(q0.coeffs().data(), q1.coeffs().data(), dq_calc.data());

    //Check right derivative
    Eigen::Matrix<double, 2, 4> ddq_dq1 = fyp.boxMinusJacobianRight(q0.coeffs().data(), q1.coeffs().data());
//std::cout << "ddq_dq1:\n" << ddq_dq1 << std::endl;

    //Perturb q1
    Eigen::Vector4d dq1(Eigen::Vector4d::Random());
    dq1 *= 1e-4;

    Eigen::Quaterniond q1p = q1;
    q1p.coeffs() += dq1;
//std::cout << "q1p: " << q1p.coeffs().transpose() << std::endl;
    Eigen::Vector2d dq_calc_p;
    fyp.boxMinus(q0.coeffs().data(), q1p.coeffs().data(), dq_calc_p.data());

    Eigen::Vector2d ddq = dq_calc_p - dq_calc;
    Eigen::Vector2d ddq_est = ddq_dq1 * dq1;

//		std::cout << "dq     : " << dq_calc.transpose() << "\ndqp    : " << dq_calc_p.transpose() <<
//				"\ndqp_est: " << (dq_calc + ddq_est).transpose() << std::endl;

    for (int i = 0; i < 2; ++i)
      EXPECT_LT(fabs(ddq(i) - ddq_est(i)), tolerance);

    //Check left derivative
    Eigen::Matrix<double, 2, 4> ddq_dq0 = fyp.boxMinusJacobianLeft(q0.coeffs().data(), q1.coeffs().data());
//std::cout << "ddq_dq0:\n" << ddq_dq0 << std::endl;

    //Perturb q0
    Eigen::Vector4d dq0(Eigen::Vector4d::Random());
    dq0 *= 1e-4;

    Eigen::Quaterniond q0p = q0;
    q0p.coeffs() += dq0;
//std::cout << "q0p: " << q0p.coeffs().transpose() << std::endl;
    Eigen::Vector2d dq0_calc_p;
    fyp.boxMinus(q0p.coeffs().data(), q1.coeffs().data(), dq0_calc_p.data());

    Eigen::Vector2d ddq0 = dq0_calc_p - dq_calc;
    Eigen::Vector2d ddq0_est = ddq_dq0 * dq0;

//		std::cout << "dq     : " << dq_calc.transpose() << "\ndqp    : " << dq_calc_p.transpose() <<
//				"\ndqp_est: " << (dq_calc + ddq_est).transpose() << std::endl;

    for (int i = 0; i < 2; ++i)
      EXPECT_LT(fabs(ddq0(i) - ddq0_est(i)), tolerance);
  }
}

TEST(fixedYawPlusMinusTest, ShouldPass) {
  for (int i = 0; i < 10; ++i) {
    FixedYawParameterization fyp;
    Eigen::Quaterniond q0(Eigen::Vector4d::Random());
    q0.normalize();

    Eigen::Vector2d dq(Eigen::Vector2d::Random());
    dq *= 1e-6;

    Eigen::Quaterniond q1;
    EXPECT_TRUE(fyp.Plus(q0.coeffs().data(), dq.data(), q1.coeffs().data()));
//std::cout << "q0: " << q0.coeffs().transpose() << "\nq1: " << q1.coeffs().transpose() << std::endl;

    Eigen::Vector2d dq_calc;
    fyp.boxMinus(q0.coeffs().data(), q1.coeffs().data(), dq_calc.data());

//		std::cout << "dq : " << dq.transpose() << "\ndqc: " << dq_calc.transpose() << "\n\n" << std::endl;

    for (int i = 0; i < 2; ++i)
      EXPECT_LT(fabs(dq(i) - dq_calc(i)), tolerance);
  }
}

int main(int argc, char **argv) {
  srand(time(NULL));
  std::cout.precision(12);

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
