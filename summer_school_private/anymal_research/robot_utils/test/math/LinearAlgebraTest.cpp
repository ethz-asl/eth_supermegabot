/*
 * LinearAlgebraTest.cpp
 *
 *  Created on: 6 Mar 2019
 *      Author: Perry Franklin
 */

#include <gtest/gtest.h>
#include <kindr/common/gtest_eigen.hpp>
#include <kindr/Core>

#include "robot_utils/math/LinearAlgebra.hpp"

TEST(LinearAlgebraTest, adaptSingularValues_identity)
{
  using namespace robot_utils;

  const Eigen::Matrix3d testInitial = Eigen::Matrix3d::Identity();

  const Eigen::Matrix3d testResult = adaptSingularValues(testInitial, 0.1);

  const Eigen::Matrix3d testExpected = Eigen::Matrix3d::Identity();

  KINDR_ASSERT_DOUBLE_MX_EQ(testResult, testExpected, 1e-7, "The function adaptSingularValues damped a matrix it should not have");
}

TEST(LinearAlgebraTest, adaptSingularValues_one_singular)
{
  using namespace robot_utils;

  Eigen::Matrix3d testInitial;
  // clang-format off
  testInitial << 1, 0, 0,
                 0, 1, 0,
                 0, 0, 0;
  // clang-format on

  const Eigen::Matrix3d testResult = adaptSingularValues(testInitial, 0.1);

  Eigen::Matrix3d testExpected;
  // clang-format off
  testExpected << 1, 0, 0,
                  0, 1, 0,
                  0, 0, 0.1;
  // clang-format on

  KINDR_ASSERT_DOUBLE_MX_EQ(testResult, testExpected, 1e-7, "");
}

TEST(LinearAlgebraTest, adaptSingularValues_two_singular)
{
  using namespace robot_utils;

  Eigen::Matrix3d testInitial;
  // clang-format off
  testInitial << 1, 0, 0,
                 0, 0, 0,
                 0, 0, 0;
  // clang-format on

  const Eigen::Matrix3d testResult = adaptSingularValues(testInitial, 0.1);

  Eigen::Matrix3d testExpected;
  // clang-format off
  testExpected << 1, 0,   0,
                  0, 0.1, 0,
                  0, 0, 0.1;
  // clang-format on

  KINDR_ASSERT_DOUBLE_MX_EQ(testResult, testExpected, 1e-7, "");
}

TEST(LinearAlgebraTest, adaptSingularValues_nonDiagonal)
{
  using namespace robot_utils;

  Eigen::Matrix3d testInitial;
  // clang-format off
  testInitial << 0, 0, 0,
                 1, 0, 0,
                 0, 1, 0;
  // clang-format on

  const Eigen::Matrix3d testResult = adaptSingularValues(testInitial, 0.1);

  Eigen::Matrix3d testExpected;
  // clang-format off
  testExpected << 0, 0, 0.1,
                  1, 0,   0,
                  0, 1,   0;
  // clang-format on

  KINDR_ASSERT_DOUBLE_MX_EQ(testResult, testExpected, 1e-7, "");
}

TEST(LinearAlgebraTest, adaptSingularValues_nonSquareMatrix)
{
  using namespace robot_utils;

  Eigen::MatrixXd testInitial(3,5);
  // clang-format off
  testInitial << 1, 0, 0, 0, 0,
                 0, 0, 0, 0, 0,
                 0, 0, 0, 0, 0;
  // clang-format on

  const Eigen::MatrixXd testResult = adaptSingularValues(testInitial, 0.1);

  Eigen::MatrixXd testExpected(3,5);
  // clang-format off
  testExpected << 1, 0,   0, 0, 0,
                  0, 0.1, 0, 0, 0,
                  0, 0, 0.1, 0, 0;
  // clang-format on

  KINDR_ASSERT_DOUBLE_MX_EQ(testResult, testExpected, 1e-7, "");
}

TEST(LinearAlgebraTest, adaptSingularValues_block)
{
  using namespace robot_utils;

  Eigen::MatrixXd testInitial(3,6);
  // clang-format off
  testInitial << 0, 1, 0, 0, 0, 0,
                 0, 0, 0, 0, 0, 0,
                 0, 0, 0, 0, 0, 0;
  // clang-format on

  const Eigen::MatrixXd testResult = adaptSingularValues(testInitial.block(0,1,3,5), 0.1);

  Eigen::MatrixXd testExpected(3,5);
  // clang-format off
  testExpected << 1, 0,   0, 0, 0,
                  0, 0.1, 0, 0, 0,
                  0, 0, 0.1, 0, 0;
  // clang-format on

  KINDR_ASSERT_DOUBLE_MX_EQ(testResult, testExpected, 1e-7, "");
}

TEST(LinearAlgebraTest, adaptSingularValues_matlabGenerated)
{
  using namespace robot_utils;

  Eigen::MatrixXd testInitial(6,7);
  // clang-format off
  testInitial << 0.616044676146639, 0.917193663829810, 0.075854289563064, 0.568823660872193, 0.311215042044805, 0.689214503140008, 0.152378018969223,
                 0.473288848902729, 0.285839018820374, 0.053950118666607, 0.469390641058206, 0.528533135506213, 0.748151592823709, 0.825816977489547,
                 0.351659507062997, 0.757200229110721, 0.530797553008973, 0.011902069501241, 0.165648729499781, 0.450541598502498, 0.538342435260057,
                 0.830828627896291, 0.753729094278495, 0.779167230102011, 0.337122644398882, 0.601981941401637, 0.083821377996933, 0.996134716626885,
                 0.585264091152724, 0.380445846975357, 0.934010684229183, 0.162182308193243, 0.262971284540144, 0.228976968716819, 0.078175528753184,
                 0.549723608291140, 0.567821640725221, 0.129906208473730, 0.794284540683907, 0.654079098476782, 0.913337361501670, 0.442678269775446;
  // clang-format on

  const Eigen::MatrixXd testResult = adaptSingularValues(testInitial, 0.5);

  Eigen::MatrixXd testExpected(6,7);
  // clang-format off
  testExpected << 0.740583240014572, 0.887684478024043, 0.031044378684138, 0.544360148909129, 0.254909891715024, 0.686094211267314, 0.153696872819307,
                  0.622521725449704, 0.226762022750917, 0.018382093688461, 0.406423813419994, 0.438469918922249, 0.795732505296816, 0.836346350485247,
                  0.262672700711049, 0.769943837233610, 0.569191388482182, 0.017545663088050, 0.197933907962969, 0.470821545751323, 0.540547644461032,
                  0.815274294080453, 0.773917806649387, 0.772150072917874, 0.363595130473493, 0.624735901258262, 0.048500150559234, 0.989742834383985,
                  0.642734924824256, 0.353503604569927, 0.923516564289500, 0.131986166456671, 0.224294449202466, 0.256370051872351, 0.083811950701861,
                  0.361669361958920, 0.628787299700791, 0.185029644223561, 0.854504680222495, 0.754730002279866, 0.882547106306890, 0.434496074446474;
  // clang-format on

  KINDR_ASSERT_DOUBLE_MX_EQ(testResult, testExpected, 1e-7, "");
}

TEST(LinearAlgebraTest, adaptSingularValues_matlabGenerated2)
{
  using namespace robot_utils;

  Eigen::MatrixXd testInitial(7,6);
  // clang-format off
  testInitial << 0.106652770180584, 0.399782649098896, 0.145538980384717, 0.622055131485066, 0.183907788282417, 0.489252638400019,
                 0.961898080855054, 0.259870402850654, 0.136068558708664, 0.350952380892271, 0.239952525664903, 0.337719409821377,
                 0.004634224134067, 0.800068480224308, 0.869292207640089, 0.513249539867053, 0.417267069084370, 0.900053846417662,
                 0.774910464711502, 0.431413827463545, 0.579704587365570, 0.401808033751942, 0.049654430325742, 0.369246781120215,
                 0.817303220653433, 0.910647594429523, 0.549860201836332, 0.075966691690842, 0.902716109915281, 0.111202755293787,
                 0.868694705363510, 0.181847028302852, 0.144954798223727, 0.239916153553658, 0.944787189721646, 0.780252068321138,
                 0.084435845510910, 0.263802916521990, 0.853031117721894, 0.123318934835166, 0.490864092468080, 0.389738836961253;
  // clang-format on

  const Eigen::MatrixXd testResult = adaptSingularValues(testInitial, 0.5);

  Eigen::MatrixXd testExpected(7,6);
  // clang-format off
  testExpected << 0.090185679425270, 0.329367454887569, 0.179281105806074, 0.759005672088866, 0.251690820487416, 0.387693405767572,
                  0.961552529181539, 0.258392783727631, 0.136776616256932, 0.353826202941206, 0.241374910502784, 0.335588252392387,
                  0.018911140886905, 0.861118235814285, 0.840037889262753, 0.394513844745841, 0.358499389812714, 0.988105386991544,
                  0.776835146950272, 0.439643992442403, 0.575760789978296, 0.385801182450355, 0.041731913828236, 0.381117077709652,
                  0.816392661528364, 0.906753937766949, 0.551725995901489, 0.083539466158593, 0.906464219366808, 0.105586967576174,
                  0.871965721860513, 0.195834275068380, 0.138252275565827, 0.212712351482919, 0.931322794592357, 0.800425754754273,
                  0.070532151674879, 0.204349105507861, 0.881520678398184, 0.238950676684873, 0.548095484525114, 0.303989113605407;
  // clang-format on

  KINDR_ASSERT_DOUBLE_MX_EQ(testResult, testExpected, 1e-7, "");
}
