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

#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include <confusion/utilities/distances.h>
//#include <confusion/utilities/rotationMatrixDistance.h>

using namespace confusion;

const double tolerance = 1e-8;

void check_vec(Eigen::Vector3d v1, Eigen::Vector3d v2, Eigen::Vector3d dv_est) {
  //Print results
//	Eigen::Vector3d v2_est = v1 + dv_est;
//	std::cout << "v1 =[" << v1.transpose() << "]" << std::endl;
//	std::cout << "v2 =[" << v2.transpose() << "]" << std::endl;
//	std::cout << "v2e=[" << v2_est.transpose() << "]" << std::endl;

  Eigen::Vector3d dv = v2 - v1;
//	std::cout << "test: " << dv(0)-dv_est(0) << "," << dv(1)-dv_est(1) << "," <<
//			dv(2)-dv_est(2) << std::endl;

  EXPECT_LT(fabs(dv(0) - dv_est(0)), tolerance);
  EXPECT_LT(fabs(dv(1) - dv_est(1)), tolerance);
  EXPECT_LT(fabs(dv(2) - dv_est(2)), tolerance);
}

TEST(quatDistance_deriv_test, ShouldPass) {
  for (int i = 0; i < 100; ++i) {
    Eigen::Quaterniond q0(Eigen::Vector4d::Random());
    q0.normalize();
    Eigen::Quaterniond q1(Eigen::Vector4d::Random());
    q1.normalize();

    Eigen::Vector3d d;
    QuatDistance(q0, q1, d.data());

    Eigen::Matrix<double, 3, 4> jl;
    calc_de_dq_left(q0, q1, jl);

    Eigen::Matrix<double, 3, 4> jr;
    calc_de_dq_right(q0, q1, jr);

    //Perturb the quaternions
    Eigen::Vector4d dq(Eigen::Vector4d::Random());
    dq *= 0.0001;
//		std::cout << "dq=" << dq.transpose() << std::endl;
    Eigen::Quaterniond q02(q0.w() + dq(0), q0.x() + dq(1), q0.y() + dq(2), q0.z() + dq(3));
    Eigen::Quaterniond q12(q1.w() + dq(0), q1.x() + dq(1), q1.y() + dq(2), q1.z() + dq(3));

    Eigen::Vector3d d1;
    QuatDistance(q02, q1, d1.data());

    Eigen::Vector3d d2;
    QuatDistance(q0, q12, d2.data());

    Eigen::Vector3d dd_1_est = jl * dq;
    Eigen::Vector3d dd_2_est = jr * dq;

    check_vec(d, d1, dd_1_est);
    check_vec(d, d2, dd_2_est);
  }
}

TEST(quatDistanceGlobalNegativeTest, ShouldPass) {
  for (int i = 0; i < 100; ++i) {
    Eigen::Quaterniond q0(Eigen::Vector4d::Random());
    q0.normalize();
    Eigen::Quaterniond q1(Eigen::Vector4d::Random());
    q1.normalize();

    Eigen::Vector3d d;
    QuatDistanceGlobal(q0, q1, d.data());

    //Negate one quaternion
    Eigen::Quaterniond q1n(-1.0 * q1.w(), -1.0 * q1.x(), -1.0 * q1.y(), -1.0 * q1.z());

    Eigen::Vector3d dn;
    QuatDistanceGlobal(q0, q1n, dn.data());

//		std::cout << "q0: " << q0.coeffs().transpose() << std::endl;
//		std::cout << "q1: " << q1.coeffs().transpose() << "; d: " << d.transpose() << std::endl;
//		std::cout << "q1n: " << q1n.coeffs().transpose() << "; dn: " << dn.transpose() << std::endl;
//		std::cout << "d: " << d.transpose() << "; dn: " << dn.transpose() << std::endl;
//		std::cout << "d norm: " << d.norm() << "; dn norm: " << dn.norm() << std::endl;

    EXPECT_LT(fabs(d(0) - dn(0)), tolerance);
    EXPECT_LT(fabs(d(1) - dn(1)), tolerance);
    EXPECT_LT(fabs(d(2) - dn(2)), tolerance);

    EXPECT_LT(d.norm(), M_PI); //Make sure the distance is of minimal magnitude

    //Test the rotation matrix distance agreement
//    Eigen::Vector3d dr = rotationMatrixDistance(q0.toRotationMatrix(), q1.toRotationMatrix());
//    EXPECT_LT(fabs(d(0) - dr(0)), tolerance);
//    EXPECT_LT(fabs(d(1) - dr(1)), tolerance);
//    EXPECT_LT(fabs(d(2) - dr(2)), tolerance);
  }
}

//TEST(quatDistanceGlobalTest, ShouldPass) {
//	for (int i=0; i<100; ++i) {
//		Eigen::Quaterniond q0(Eigen::Vector4d::Random()); q0.normalize();
//		Eigen::Vector3d dq(Eigen::Vector3d::Random());
////		dq.normalize();
////		dq *= M_PI/;
//
//		Eigen::Quaterniond q1 = quaternionBoxPlus(q0, dq);
//
//		Eigen::Vector3d dn;
//		QuatDistanceGlobal(q0,q1,dn.data());
//	}
//}

int main(int argc, char **argv) {
  srand(time(NULL));
  std::cout.precision(8);

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
