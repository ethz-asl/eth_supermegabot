#include <gtest/gtest.h>
#include <stdlib.h> //rand
#include <ctime> //time
#include <math.h> //isnan

#include <kindr/common/gtest_eigen.hpp>
#include <kindr/Core>

#include <robot_utils/controllers/DiscretePidEigen.hpp>

TEST(DiscretePidEigen, DiscretePidEigenTest)
{
  std::srand(std::time(NULL));

  /*
   * First, check instantiation and assignment of parameters.
   */

  //instantiate with fixed size matrix
  const int rows = 6;
  const int cols = 8;

  Eigen::Matrix<double,rows,cols> zerosFixedSize;
  zerosFixedSize.setZero();

  robot_utils::DiscretePidEigen fixedSizePid(
      zerosFixedSize, zerosFixedSize, zerosFixedSize, zerosFixedSize, zerosFixedSize);

  Eigen::Matrix<double,rows,cols> pGains, iGains, dGains, cmdMax, cmdMin;
  pGains.setRandom();
  pGains = pGains.cwiseAbs();//ensure positivity
  iGains.setRandom();
  iGains = iGains.cwiseAbs();
  dGains.setRandom();
  dGains = dGains.cwiseAbs();
  cmdMax.setRandom();
  cmdMax = cmdMax.cwiseAbs();
  cmdMin = -cmdMax;

  //set gains
  fixedSizePid.setPGains(pGains);
  fixedSizePid.setIGains(iGains);
  fixedSizePid.setDGains(dGains);
  fixedSizePid.setCmdMax(cmdMax);
  fixedSizePid.setCmdMin(cmdMin);

  //check init method/constructor:
  robot_utils::DiscretePidEigen fixedSizePidInit(
      pGains, iGains, dGains, cmdMax, cmdMin);

  //try same with dynamic size
  Eigen::MatrixXd pGainsDyn, iGainsDyn, dGainsDyn, iMaxDyn, iMinDyn, cmdMaxDyn, cmdMinDyn;
  pGainsDyn = pGains;
  iGainsDyn = iGains;
  dGainsDyn = dGains;
  cmdMaxDyn = cmdMax;
  cmdMinDyn = cmdMin;
  robot_utils::DiscretePidEigen dynamicSizePid(
        pGainsDyn, iGainsDyn, dGainsDyn, cmdMaxDyn, cmdMinDyn);

  //check getter methods and assignment
  KINDR_ASSERT_DOUBLE_MX_EQ(pGains, fixedSizePid.getPGains(), 1.0e-9, "");
  KINDR_ASSERT_DOUBLE_MX_EQ(iGains, fixedSizePid.getIGains(), 1.0e-9, "");
  KINDR_ASSERT_DOUBLE_MX_EQ(dGains, fixedSizePid.getDGains(), 1.0e-9, "");
  KINDR_ASSERT_DOUBLE_MX_EQ(cmdMax, fixedSizePid.getCmdMax(), 1.0e-9, "");
  KINDR_ASSERT_DOUBLE_MX_EQ(cmdMin, fixedSizePid.getCmdMin(), 1.0e-9, "");

  KINDR_ASSERT_DOUBLE_MX_EQ(pGains, fixedSizePidInit.getPGains(), 1.0e-9, "");
  KINDR_ASSERT_DOUBLE_MX_EQ(iGains, fixedSizePidInit.getIGains(), 1.0e-9, "");
  KINDR_ASSERT_DOUBLE_MX_EQ(dGains, fixedSizePidInit.getDGains(), 1.0e-9, "");
  KINDR_ASSERT_DOUBLE_MX_EQ(cmdMax, fixedSizePidInit.getCmdMax(), 1.0e-9, "");
  KINDR_ASSERT_DOUBLE_MX_EQ(cmdMin, fixedSizePidInit.getCmdMin(), 1.0e-9, "");

  KINDR_ASSERT_DOUBLE_MX_EQ(pGains, dynamicSizePid.getPGains(), 1.0e-9, "");
  KINDR_ASSERT_DOUBLE_MX_EQ(iGains, dynamicSizePid.getIGains(), 1.0e-9, "");
  KINDR_ASSERT_DOUBLE_MX_EQ(dGains, dynamicSizePid.getDGains(), 1.0e-9, "");
  KINDR_ASSERT_DOUBLE_MX_EQ(cmdMax, dynamicSizePid.getCmdMax(), 1.0e-9, "");
  KINDR_ASSERT_DOUBLE_MX_EQ(cmdMin, dynamicSizePid.getCmdMin(), 1.0e-9, "");


  /*
   * Check zero error input.
   */
  Eigen::MatrixXd error, command, commandExpected;

  commandExpected.setZero(rows,cols);
  error.setZero(rows, cols);


  double dt = (static_cast<double>(rand())/static_cast<double>(RAND_MAX));
  //Update 1
  fixedSizePid.update<Eigen::MatrixXd>(error, command, dt);
  KINDR_ASSERT_DOUBLE_MX_EQ(command, commandExpected, 1.0e-9, "");
  //Update 2
  fixedSizePid.update<Eigen::MatrixXd>(error, command, dt);
  KINDR_ASSERT_DOUBLE_MX_EQ(command, commandExpected, 1.0e-9, "");
  //Update 3
  fixedSizePid.update<Eigen::MatrixXd>(error, command, dt);
  KINDR_ASSERT_DOUBLE_MX_EQ(command, commandExpected, 1.0e-9, "");
}
