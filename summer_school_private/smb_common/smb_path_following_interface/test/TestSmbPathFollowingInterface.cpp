//
// Created by johannes on 02.05.19.
//
#include <gtest/gtest.h>
#include <smb_path_following_interface/SmbPathFollowingConversions.h>
#include <smb_path_following_interface/SmbPathFollowingInterface.h>
#include <cmath>

using namespace smb_path_following;

TEST(SmbPathFollowingInterfaceTests, instantiation) {
  SmbPathFollowingInterface mmInterface;

  ASSERT_TRUE(true);
}

TEST(SmbPathFollowingConversions, mpcStateToStateConversions) {
  for (int i = 0; i < 10; i++) {
    SmbPathFollowingConversions::state_vector_t stateVector;
    SmbPathFollowingConversions::state_vector_t stateVectorCheck;
    stateVector.setRandom();
    kindr::HomTransformQuatD pose;
    SmbPathFollowingConversions::readMpcState(stateVector, pose);
    SmbPathFollowingConversions::writeMpcState(stateVectorCheck, pose);

    ASSERT_NEAR(stateVector[0], stateVectorCheck[0], 1e-6);
    ASSERT_NEAR(stateVector[1], stateVectorCheck[1], 1e-6);
    ASSERT_NEAR(stateVector[2], stateVectorCheck[2], 1e-6);
  }
}

/*TEST(SmbPathFollowingConversions, kindrEulerAngleConversions) {
  //0.753279 -3.24419e-06 6.28533e-06 -0.657701
  kindr::RotationQuaternionD rot1;
  rot1.w() = 0.753279;
  rot1.x() = -3.24419e-06;
  rot1.y() = 6.28533e-06;
  rot1.z() = -0.657701;
  kindr::EulerAnglesRpyD euler1(rot1);
  std::cout << "euler1:" << euler1 << std::endl;

  //0.752175 -1.08563e-05 -7.14059e-06 -0.658963
  kindr::RotationQuaternionD rot2;
  rot2.w() = 0.752175;
  rot2.x() = -1.08563e-05;
  rot2.y() = -7.14059e-06;
  rot2.z() = -0.658963;
  kindr::EulerAnglesRpyD euler2(rot2);
  std::cout << "euler2:" << euler2 << std::endl;
}*/

TEST(SmbPathFollowingConversions, mpcPoseToStateConversion){
  kindr::HomTransformQuatD pose1;
  kindr::RotationQuaternionD rot1;
  rot1.w() = 0.753279;
  rot1.x() = -3.24419e-06;
  rot1.y() = 6.28533e-06;
  rot1.z() = -0.657701;
  pose1.getRotation() = rot1;
  SmbPathFollowingConversions::state_vector_t state1;
  SmbPathFollowingConversions::writeMpcState(state1, pose1);

  kindr::HomTransformQuatD pose2;
  kindr::RotationQuaternionD rot2;
  rot2.w() = 0.752175;
  rot2.x() = -1.08563e-05;
  rot2.y() = -7.14059e-06;
  rot2.z() = -0.658963;
  pose2.getRotation() = rot2;
  SmbPathFollowingConversions::state_vector_t state2;
  SmbPathFollowingConversions::writeMpcState(state2, pose2);

  ASSERT_NEAR(state1[2], state2[2], 1e-2);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
