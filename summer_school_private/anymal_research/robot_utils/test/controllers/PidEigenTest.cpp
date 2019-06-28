#include <gtest/gtest.h>
#include <stdlib.h> //rand
#include <ctime> //time
#include <math.h> //isnan

#include <kindr/common/gtest_eigen.hpp>
#include <kindr/Core>

#include <robot_utils/controllers/PidEigen.hpp>

TEST(PidEigen, PiDEigenTest)
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

  robot_utils::PidEigen fixedSizePid(
      zerosFixedSize, zerosFixedSize, zerosFixedSize, zerosFixedSize, zerosFixedSize, zerosFixedSize, zerosFixedSize);

  Eigen::Matrix<double,rows,cols> pGains, iGains, dGains, iMax, iMin, cmdMax, cmdMin;
  pGains.setRandom();
  pGains = pGains.cwiseAbs();//ensure positivity
  iGains.setRandom();
  iGains = iGains.cwiseAbs();
  dGains.setRandom();
  dGains = dGains.cwiseAbs();
  iMax.setRandom();
  iMax = iMax.cwiseAbs();
  iMin = -iMax;
  cmdMax.setRandom();
  cmdMax = cmdMax.cwiseAbs();
  cmdMin = -cmdMax;

  //set gains
  fixedSizePid.setPGains(pGains);
  fixedSizePid.setIGains(iGains);
  fixedSizePid.setDGains(dGains);
  fixedSizePid.setIMax(iMax);
  fixedSizePid.setIMin(iMin);
  fixedSizePid.setCmdMax(cmdMax);
  fixedSizePid.setCmdMin(cmdMin);

  //check init method/constructor:
  robot_utils::PidEigen fixedSizePidInit(
      pGains, iGains, dGains, iMax, iMin, cmdMax, cmdMin);

  //try same with dynamic size
  Eigen::MatrixXd pGainsDyn, iGainsDyn, dGainsDyn, iMaxDyn, iMinDyn, cmdMaxDyn, cmdMinDyn;
  pGainsDyn = pGains;
  iGainsDyn = iGains;
  dGainsDyn = dGains;
  iMaxDyn = iMax;
  iMinDyn = iMin;
  cmdMaxDyn = cmdMax;
  cmdMinDyn = cmdMin;
  robot_utils::PidEigen dynamicSizePid(
        pGainsDyn, iGainsDyn, dGainsDyn, iMaxDyn, iMinDyn, cmdMaxDyn, cmdMinDyn);

  //check getter methods and assignment
  KINDR_ASSERT_DOUBLE_MX_EQ(pGains, fixedSizePid.getPGains(), 1.0e-9, "");
  KINDR_ASSERT_DOUBLE_MX_EQ(iGains, fixedSizePid.getIGains(), 1.0e-9, "");
  KINDR_ASSERT_DOUBLE_MX_EQ(dGains, fixedSizePid.getDGains(), 1.0e-9, "");
  KINDR_ASSERT_DOUBLE_MX_EQ(iMax, fixedSizePid.getIMax(), 1.0e-9, "");
  KINDR_ASSERT_DOUBLE_MX_EQ(iMin, fixedSizePid.getIMin(), 1.0e-9, "");
  KINDR_ASSERT_DOUBLE_MX_EQ(cmdMax, fixedSizePid.getCmdMax(), 1.0e-9, "");
  KINDR_ASSERT_DOUBLE_MX_EQ(cmdMin, fixedSizePid.getCmdMin(), 1.0e-9, "");

  KINDR_ASSERT_DOUBLE_MX_EQ(pGains, fixedSizePidInit.getPGains(), 1.0e-9, "");
  KINDR_ASSERT_DOUBLE_MX_EQ(iGains, fixedSizePidInit.getIGains(), 1.0e-9, "");
  KINDR_ASSERT_DOUBLE_MX_EQ(dGains, fixedSizePidInit.getDGains(), 1.0e-9, "");
  KINDR_ASSERT_DOUBLE_MX_EQ(iMax, fixedSizePidInit.getIMax(), 1.0e-9, "");
  KINDR_ASSERT_DOUBLE_MX_EQ(iMin, fixedSizePidInit.getIMin(), 1.0e-9, "");
  KINDR_ASSERT_DOUBLE_MX_EQ(cmdMax, fixedSizePidInit.getCmdMax(), 1.0e-9, "");
  KINDR_ASSERT_DOUBLE_MX_EQ(cmdMin, fixedSizePidInit.getCmdMin(), 1.0e-9, "");

  KINDR_ASSERT_DOUBLE_MX_EQ(pGains, dynamicSizePid.getPGains(), 1.0e-9, "");
  KINDR_ASSERT_DOUBLE_MX_EQ(iGains, dynamicSizePid.getIGains(), 1.0e-9, "");
  KINDR_ASSERT_DOUBLE_MX_EQ(dGains, dynamicSizePid.getDGains(), 1.0e-9, "");
  KINDR_ASSERT_DOUBLE_MX_EQ(iMax, dynamicSizePid.getIMax(), 1.0e-9, "");
  KINDR_ASSERT_DOUBLE_MX_EQ(iMin, dynamicSizePid.getIMin(), 1.0e-9, "");
  KINDR_ASSERT_DOUBLE_MX_EQ(cmdMax, dynamicSizePid.getCmdMax(), 1.0e-9, "");
  KINDR_ASSERT_DOUBLE_MX_EQ(cmdMin, dynamicSizePid.getCmdMin(), 1.0e-9, "");


  /*
   * Check two update steps.
   */
  Eigen::MatrixXd error1, error2, command1, command1Expected, command2, command2Expected, pTerm, iTerm, iErr, dTerm;

  //Update 1
  command1Expected.setZero(rows,cols);
  error1.setRandom(rows, cols);
  iErr.setZero(rows,cols);


  double dt = (static_cast<double>(rand())/static_cast<double>(RAND_MAX));
  fixedSizePid.update<Eigen::MatrixXd>(error1, command1, dt);
  pTerm = pGains.cwiseProduct(error1);
  iErr = dt*error1;
  iTerm = iGains.cwiseProduct(iErr);
  dTerm = dGains.cwiseProduct(error1/dt);
  for(int i=0; i<rows; i++){
    for(int j=0; j<cols; j++){
      if(iTerm(i,j) > iMax(i,j)){
        iTerm(i,j) = iMax(i,j);
        iErr(i,j) = iMax(i,j)/iGains(i,j);
      }

      if(iTerm(i,j) < iMin(i,j)){
        iTerm(i,j) = iMin(i,j);
        iErr(i,j) = iMin(i,j)/iGains(i,j);
      }

      command1Expected(i,j) = -pTerm(i,j) - iTerm(i,j) - dTerm(i,j);
      if(command1Expected(i,j) > cmdMax(i,j))
        command1Expected(i,j) = cmdMax(i,j);
      if(command1Expected(i,j) < cmdMin(i,j))
        command1Expected(i,j) = cmdMin(i,j);
    }
  }

  KINDR_ASSERT_DOUBLE_MX_EQ(command1, command1Expected, 1.0e-9, "");

  //Update 2
  error2.setRandom(rows, cols);
  command2Expected.setZero(rows,cols);
  dt = (static_cast<double>(rand())/static_cast<double>(RAND_MAX));
  fixedSizePid.update(error2, command2, dt);
  pTerm = pGains.cwiseProduct(error2);
  iTerm = iGains.cwiseProduct(dt*error2 + iErr);
  dTerm = dGains.cwiseProduct((error2-error1)/dt);
  for(int i=0; i<rows; i++){
    for(int j=0; j<cols; j++){
      if(iTerm(i,j) > iMax(i,j))
        iTerm(i,j) = iMax(i,j);
      if(iTerm(i,j) < iMin(i,j))
        iTerm(i,j) = iMin(i,j);

      command2Expected(i,j) = -pTerm(i,j) - iTerm(i,j) - dTerm(i,j);
      if(command2Expected(i,j) > cmdMax(i,j))
        command2Expected(i,j) = cmdMax(i,j);
      if(command2Expected(i,j) < cmdMin(i,j))
        command2Expected(i,j) = cmdMin(i,j);
    }
  }

  KINDR_ASSERT_DOUBLE_MX_EQ(command2, command2Expected, 1.0e-9, "");


  /*
   * Finally check that zero i-Gain works as expected
   */
  iGains.setOnes();
  iGains(0,0) = 0.0;
  fixedSizePid.setIGains(iGains);
  iMax.setOnes();
  iMin.setOnes();
  iMin *= -1.0;
  fixedSizePid.setIMax(iMax);
  fixedSizePid.setIMin(iMin);

  fixedSizePid.reset();

  error1.setOnes();
  error1*=2.0;
  dt = 1.0;
  fixedSizePid.update(error1, command1, dt);


  Eigen::MatrixXd pErr, dErr, pErrExpected, iErrExpected, dErrExpected;
  fixedSizePid.getErrors(pErr,iErr,dErr);
  pErrExpected = error1;
  iErrExpected = Eigen::MatrixXd::Ones(rows,cols);
  iErrExpected(0,0) = 0.0;
  dErrExpected = error1/dt;

  KINDR_ASSERT_DOUBLE_MX_EQ(pErr, pErrExpected, 1.0e-9, "");
  KINDR_ASSERT_DOUBLE_MX_EQ(iErr, iErrExpected, 1.0e-9, "");
  KINDR_ASSERT_DOUBLE_MX_EQ(dErr, dErrExpected, 1.0e-9, "");
  ASSERT_FALSE(iErr.hasNaN());
  ASSERT_FALSE(command1.hasNaN());

  fixedSizePid.update(error2, command2, dt);
  ASSERT_FALSE(command2.hasNaN());
}
