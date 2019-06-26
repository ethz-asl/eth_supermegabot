/*
 * QuaternionTest.cpp
 *
 *  Created on: Jun 16, 2016
 *      Author: Christian Gehring
 */


#include <gtest/gtest.h>
#include <any_rbdl/rbdl.h>
#include <kindr/Core>
#include "kindr/rotations/gtest_rotations.hpp"

namespace kindr {

template<>
class RotationConversion<RigidBodyDynamics::Math::Quaternion, Eigen::Vector3d, double> {
  typedef RigidBodyDynamics::Math::Quaternion Rotation;
 public:

  inline static void convertToOtherRotation(Rotation& out, const kindr::RotationQuaternion<double>& in) {
    kindr::RotationQuaternion<double> in2 = in;
    out = Rotation(in2.x(), in2.y(), in2.z(), in2.w());
  }

  inline static void convertToKindr(kindr::RotationQuaternion<double>& out, const Rotation& in ) {
    out =  kindr::RotationQuaternion<double>(in.w(), in.x(), in.y(), in.z());
  }

  inline static void convertToOtherVelocityVector(Eigen::Vector3d& out, Rotation& rot, const Eigen::Matrix<double,3,1>& in) {
    out = in;
  }

  inline static void getRotationMatrixFromRotation(Eigen::Matrix3d& rotationMatrix, const Rotation& quaternion) {
    rotationMatrix = quaternion.toMatrix().transpose();
  }

  inline static void concatenate(Rotation& res,  const Rotation& rot1,  const Rotation& rot2) {
    res = rot1*rot2;
  }

  inline static void rotateVector(Eigen::Matrix<double,3,1>& A_r, const Rotation& rotationBToA, const Eigen::Matrix<double,3,1>& B_r) {
    A_r = rotationBToA.rotate(B_r);
  }

  inline static void boxPlus(Rotation& res, const Rotation& quaternion, const Eigen::Vector3d& velocity) {
    Rotation quat = quaternion;
    res = quat.timeStep(velocity, 1.0);
  }

/*  inline static void testRotation(const Rotation& expected, const Rotation& actual) {
    EXPECT_NEAR(expected(0), actual(0), 1.0e-6) << " expected: " << expected.transpose() << " actual: " << actual.transpose();
    EXPECT_NEAR(expected(1), actual(1), 1.0e-6) << " expected: " << expected.transpose() << " actual: " << actual.transpose();
    EXPECT_NEAR(expected(2), actual(2), 1.0e-6) << " expected: " << expected.transpose() << " actual: " << actual.transpose();
    EXPECT_NEAR(expected(3), actual(3), 1.0e-6) << " expected: " << expected.transpose() << " actual: " << actual.transpose();
  }*/

};

} // namespace kindr
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

TEST(QuaternionTest, Concatenation) {
  kindr::ConventionTest<RigidBodyDynamics::Math::Quaternion, Eigen::Vector3d, double>::testConcatenation();
}

TEST(QuaternionTest, Rotation) {
  kindr::ConventionTest<RigidBodyDynamics::Math::Quaternion, Eigen::Vector3d, double>::testRotationMatrix();
}

TEST(QuaternionTest, BoxPlus) {
  kindr::ConventionTest<RigidBodyDynamics::Math::Quaternion, Eigen::Vector3d, double>::testBoxPlus();
}

TEST(QuaternionTest, GeometricalInterpretation) {
  kindr::ConventionTest<RigidBodyDynamics::Math::Quaternion, Eigen::Vector3d, double>::testGeometricalInterpretation();
}

TEST(KindrRbdlTest, RotateQuatVsMatrix) {
  kindr::RotationQuaternionD input1(kindr::EulerAnglesZyxD(0.1, 0.2, 0.3));
  double w1 = input1.w();
  double x1 = input1.x();
  double y1 = input1.y();
  double z1 = input1.z();
  kindr::RotationQuaternionD kindrQuat(w1, x1, y1, z1);
  RigidBodyDynamics::Math::Quaternion rbdlQuat(x1, y1, z1, w1);
  kindr::RotationMatrixD kindrMat(kindrQuat);
  Eigen::Matrix3d kindrMatrix = kindrMat.matrix();
  Eigen::Matrix3d rbdlMatrix =  rbdlQuat.toMatrix();

  Eigen::Vector3d A_r(0.6, 0.1, 1.4);
  Eigen::Vector3d B_r_kindrMatrixMult;
  Eigen::Vector3d B_r_kindrMatrixRot;
  Eigen::Vector3d B_r_kindrQuat;
  Eigen::Vector3d B_r_rbdlMat;
  Eigen::Vector3d B_r_rbdlQuat;

  B_r_kindrMatrixMult = kindrMatrix*A_r;
  B_r_kindrMatrixRot = kindrMat.rotate(A_r);
  B_r_kindrQuat = kindrQuat.rotate(A_r);
  B_r_rbdlMat = rbdlMatrix.transpose()*A_r;
  B_r_rbdlQuat = rbdlQuat.rotate(A_r);
  KINDR_ASSERT_DOUBLE_MX_EQ(B_r_kindrMatrixMult, B_r_kindrMatrixRot, 1.0e-3, "kindr_matrix_rotate_mult");
  KINDR_ASSERT_DOUBLE_MX_EQ(B_r_kindrQuat, B_r_kindrMatrixRot, 1.0e-3, "kindr_quat_matrix_rotate");
  KINDR_ASSERT_DOUBLE_MX_EQ(B_r_rbdlQuat, B_r_rbdlMat, 1.0e-3, "rbdl_quat_matrix");
}
