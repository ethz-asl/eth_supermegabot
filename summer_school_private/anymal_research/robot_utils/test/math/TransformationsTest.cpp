/*!
* @file     TransformationsTest.cpp
* @author   Christian Gehring
* @date     May 22, 2016
* @brief
*/

#include <gtest/gtest.h>
#include <kindr/common/gtest_eigen.hpp>
#include <kindr/Core>

#include "robot_utils/math/Transformations.hpp"

TEST(TransformationsTest, getPerspectiveTransform_identity)
{
  using namespace robot_utils;

  std::vector<Eigen::Vector3d> src;
  std::vector<Eigen::Vector3d> dst;


  src.push_back(Eigen::Vector3d(0.0, 0.0, 1.0));
  dst.push_back(Eigen::Vector3d(0.0, 0.0, 1.0));

  src.push_back(Eigen::Vector3d(0.0, 1.0, 1.0));
  dst.push_back(Eigen::Vector3d(0.0, 1.0, 1.0));

  src.push_back(Eigen::Vector3d(1.0, 1.0, 1.0));
  dst.push_back(Eigen::Vector3d(1.0, 1.0, 1.0));

  src.push_back(Eigen::Vector3d(1.0, 0.0, 1.0));
  dst.push_back(Eigen::Vector3d(1.0, 0.0, 1.0));

  Eigen::Matrix3d M = getPerspectiveTransform(src, dst);

  std::vector<Eigen::Vector3d>  calcDest(4);
  for (int i=0; i < 4; i++) {
    calcDest[i] = M*src[i];
    calcDest[i] /= calcDest[i].z();
    KINDR_ASSERT_DOUBLE_MX_EQ(calcDest[i], dst[i], 1.0, "identity");
  }
}

TEST(TransformationsTest, getPerspectiveTransform_translation)
{
  using namespace robot_utils;

  std::vector<Eigen::Vector3d> src;
  std::vector<Eigen::Vector3d> dst;

  src.push_back(Eigen::Vector3d(0.0, 0.0, 1.0));
  src.push_back(Eigen::Vector3d(0.0, 1.0, 1.0));
  src.push_back(Eigen::Vector3d(1.0, 1.0, 1.0));
  src.push_back(Eigen::Vector3d(1.0, 0.0, 1.0));

  for (int i=0; i < 4; i++) {
    dst.push_back(src[i]+Eigen::Vector3d(0.1, 0.2, 0.0));
  }

  Eigen::Matrix3d M = getPerspectiveTransform(src, dst);

  std::vector<Eigen::Vector3d>  calcDest(4);
  for (int i=0; i < 4; i++) {
    calcDest[i] = M*src[i];
    calcDest[i] /= calcDest[i].z();
    KINDR_ASSERT_DOUBLE_MX_EQ(calcDest[i], dst[i], 1.0, "translation");
  }
}

TEST(TransformationsTest, getPerspectiveTransform_rotation)
{
  using namespace robot_utils;

  std::vector<Eigen::Vector3d> src;
  std::vector<Eigen::Vector3d> dst;

  kindr::EulerAnglesZyxPD rot(10.0/180.0*M_PI, 20.0/180.0*M_PI, 30.0/180.0*M_PI);

  src.push_back(Eigen::Vector3d(0.0, 0.0, 1.0));
  src.push_back(Eigen::Vector3d(0.0, 1.0, 1.0));
  src.push_back(Eigen::Vector3d(1.0, 1.0, 1.0));
  src.push_back(Eigen::Vector3d(1.0, 0.0, 1.0));

  for (int i=0; i < 4; i++) {
    dst.push_back(rot.rotate(src[i]));
    dst.back().z() = 1.0;
  }

  Eigen::Matrix3d M = getPerspectiveTransform(src, dst);

  std::vector<Eigen::Vector3d>  calcDest(4);
  for (int i=0; i < 4; i++) {
    calcDest[i] = M*src[i];
    calcDest[i] /= calcDest[i].z();
    KINDR_ASSERT_DOUBLE_MX_EQ(calcDest[i], dst[i], 1.0, "rotation");
  }
}

TEST(TransformationsTest, getPerspectiveTransform_scale)
{
  using namespace robot_utils;

  std::vector<Eigen::Vector3d> src;
  std::vector<Eigen::Vector3d> dst;

  kindr::EulerAnglesZyxPD rot(10.0/180.0*M_PI, 20.0/180.0*M_PI, 30.0/180.0*M_PI);

  src.push_back(Eigen::Vector3d(0.0, 0.0, 1.0));
  src.push_back(Eigen::Vector3d(0.0, 1.0, 1.0));
  src.push_back(Eigen::Vector3d(1.0, 1.0, 1.0));
  src.push_back(Eigen::Vector3d(1.0, 0.0, 1.0));

  Eigen::Matrix3d scale = Eigen::Matrix3d::Zero();
  scale(0,0) = 0.1;
  scale(1,1) = 0.2;

  for (int i=0; i < 4; i++) {
    dst.push_back(scale*src[i]);
    dst.back().z() = 1.0;
  }

  Eigen::Matrix3d M = getPerspectiveTransform(src, dst);

  std::vector<Eigen::Vector3d>  calcDest(4);
  for (int i=0; i < 4; i++) {
    calcDest[i] = M*src[i];
    calcDest[i] /= calcDest[i].z();
    KINDR_ASSERT_DOUBLE_MX_EQ(calcDest[i], dst[i], 1.0, "scale");
  }
}


TEST(TransformationsTest, getAffineTransform_identity)
{
  using namespace robot_utils;

  std::vector<Eigen::Vector3d> src;
  std::vector<Eigen::Vector3d> dst;

  src.push_back(Eigen::Vector3d(0.0, 0.0, 1.0));
  dst.push_back(Eigen::Vector3d(0.0, 0.0, 1.0));

  src.push_back(Eigen::Vector3d(0.0, 1.0, 1.0));
  dst.push_back(Eigen::Vector3d(0.0, 1.0, 1.0));

  src.push_back(Eigen::Vector3d(1.0, 1.0, 1.0));
  dst.push_back(Eigen::Vector3d(1.0, 1.0, 1.0));

  Eigen::Matrix3d M = getAffineTransform(src, dst);

  std::vector<Eigen::Vector3d>  calcDest(3);
  for (int i=0; i < 3; i++) {
    calcDest[i] = M*src[i];
    KINDR_ASSERT_DOUBLE_MX_EQ(calcDest[i], dst[i], 1.0, "identity");
  }
}

TEST(TransformationsTest, getAffineTransform_translation)
{
  using namespace robot_utils;

  std::vector<Eigen::Vector3d> src;
  std::vector<Eigen::Vector3d> dst;

  src.push_back(Eigen::Vector3d(0.0, 0.0, 1.0));
  src.push_back(Eigen::Vector3d(0.0, 1.0, 1.0));
  src.push_back(Eigen::Vector3d(1.0, 1.0, 1.0));

  for (int i=0; i < 3; i++) {
    dst.push_back(src[i]+Eigen::Vector3d(0.1, 0.2, 0.0));
  }

  Eigen::Matrix3d M = getAffineTransform(src, dst);

  std::vector<Eigen::Vector3d>  calcDest(3);
  for (int i=0; i < 3; i++) {
    calcDest[i] = M*src[i];
    KINDR_ASSERT_DOUBLE_MX_EQ(calcDest[i], dst[i], 1.0, "translation");
  }
}

TEST(TransformationsTest, getAffineTransform_rotation)
{
  using namespace robot_utils;

  std::vector<Eigen::Vector3d> src;
  std::vector<Eigen::Vector3d> dst;

  kindr::EulerAnglesZyxPD rot(10.0/180.0*M_PI, 20.0/180.0*M_PI, 30.0/180.0*M_PI);

  src.push_back(Eigen::Vector3d(0.0, 0.0, 1.0));
  src.push_back(Eigen::Vector3d(0.0, 1.0, 1.0));
  src.push_back(Eigen::Vector3d(1.0, 1.0, 1.0));

  for (int i=0; i < 3; i++) {
    dst.push_back(rot.rotate(src[i]));
    dst.back().z() = 1.0;
  }

  Eigen::Matrix3d M = getAffineTransform(src, dst);

  std::vector<Eigen::Vector3d>  calcDest(3);
  for (int i=0; i < 3; i++) {
    calcDest[i] = M*src[i];
    KINDR_ASSERT_DOUBLE_MX_EQ(calcDest[i], dst[i], 1.0, "rotation");
  }
}

TEST(TransformationsTest, getAffineTransform_scale)
{
  using namespace robot_utils;

  std::vector<Eigen::Vector3d> src;
  std::vector<Eigen::Vector3d> dst;

  kindr::EulerAnglesZyxPD rot(10.0/180.0*M_PI, 20.0/180.0*M_PI, 30.0/180.0*M_PI);

  src.push_back(Eigen::Vector3d(0.0, 0.0, 1.0));
  src.push_back(Eigen::Vector3d(0.0, 1.0, 1.0));
  src.push_back(Eigen::Vector3d(1.0, 1.0, 1.0));

  Eigen::Matrix3d scale = Eigen::Matrix3d::Zero();
  scale(0,0) = 0.1;
  scale(1,1) = 0.2;

  for (int i=0; i < 3; i++) {
    dst.push_back(scale*src[i]);
    dst.back().z() = 1.0;
  }

  Eigen::Matrix3d M = getAffineTransform(src, dst);

  std::vector<Eigen::Vector3d>  calcDest(3);
  for (int i=0; i < 3; i++) {
    calcDest[i] = M*src[i];
    KINDR_ASSERT_DOUBLE_MX_EQ(calcDest[i], dst[i], 1.0, "scale");
  }
}
