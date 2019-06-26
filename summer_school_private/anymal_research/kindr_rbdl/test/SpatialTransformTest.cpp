/*
 * SpatialTransformTest.cpp
 *
 *  Created on: Jun 18, 2016
 *      Author: Christian Gehring
 */

// test
#include <gtest/gtest.h>

// kindr
#include <kindr/Core>
#include <kindr/rotations/gtest_rotations.hpp>

// rbdl
#include <any_rbdl/rbdl.h>


namespace kindr {

template<>
class RotationConversion<RigidBodyDynamics::Math::SpatialTransform, Eigen::Vector3d, double> {
  typedef double Scalar;
  typedef RigidBodyDynamics::Math::SpatialTransform Rotation;
 public:
  inline static void convertToOtherRotation(Rotation& rotation, const kindr::RotationQuaternion<Scalar>& quaternionIn) {
    Eigen::Matrix3d matrix = kindr::RotationMatrixD(quaternionIn).matrix();
    rotation = Rotation(matrix, Eigen::Vector3d::Zero());
  }

  inline static void convertToKindr(kindr::RotationQuaternion<Scalar>& quaternion, const Rotation& otherRotation) {
    // Implement quaternionOut = f(quaternionIn);
    Eigen::Matrix<Scalar,3,3> rotationMatrix;
    getRotationMatrixFromRotation(rotationMatrix, otherRotation);
    quaternion(kindr::RotationMatrix<Scalar>(rotationMatrix));
  }

  inline static void convertToOtherVelocityVector(Eigen::Vector3d& velocityOut, const Rotation& rot, const Eigen::Matrix<Scalar,3,1>& velocityIn) {
    // Implement velocityOut = g(velocityIn, rot);
    velocityOut = velocityIn;
  }

  inline static void concatenate(Rotation& res,  const Rotation& rot1,  const Rotation& rot2) {
    res = rot2*rot1;
  }

  inline static void getRotationMatrixFromRotation(Eigen::Matrix<Scalar,3,3>& rotationMatrix, const Rotation& rotation) {
    rotationMatrix = rotation.E;
  }

  inline static void rotateVector(Eigen::Matrix<Scalar,3,1>& A_r, const Rotation& rotationBToA, const Eigen::Matrix<Scalar,3,1>& B_r) {
    //RigidBodyDynamics::Math::SpatialVector vector()
    A_r = rotationBToA.E*B_r;
  }

  inline static void boxPlus(Rotation& res, const Rotation& rotation, const Eigen::Vector3d& velocity) {
    // Implement res = rotation.boxPlus(vector);

  }

  inline static void testRotation(const Rotation& expected, const Rotation& actual) {
    Eigen::Matrix3d actualM = actual.E;
    Eigen::Matrix3d expectedM = expected.E;
    KINDR_ASSERT_DOUBLE_MX_EQ(expectedM, actualM, 1e-4, "constructor");
  }
};

}

TEST(SpatialTransformationTest, GeometricalInterpretation)
{
 kindr::ConventionTest<RigidBodyDynamics::Math::SpatialTransform, Eigen::Vector3d, double>::testGeometricalInterpretation();
}

TEST(SpatialTransformationTest, Concatenation)
{
 kindr::ConventionTest<RigidBodyDynamics::Math::SpatialTransform, Eigen::Vector3d, double>::testConcatenation();
}


TEST(SpatialTransformationTest, Rotation)
{
 kindr::ConventionTest<RigidBodyDynamics::Math::SpatialTransform, Eigen::Vector3d, double>::testRotationMatrix();
}

TEST(SpatialTransformationTest, DISABLED_BoxPlus)
{
 kindr::ConventionTest<RigidBodyDynamics::Math::SpatialTransform, Eigen::Vector3d, double>::testBoxPlus();
}
