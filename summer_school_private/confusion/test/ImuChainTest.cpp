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

//This tells the IMU cost function to use a constant weighting, since we are making an
//approximation that this weighting doesn't change wrt the parameters, even though it does
#define IMU_UNIT_TEST
//#define IMU_COST_DEBUG

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include "confusion/models/ImuChain.h"
#include "confusion/State.h"
#include "confusion/StaticParameterVector.h"
#include <confusion/utilities/Pose.h>

using namespace confusion;

const double tolerance = 1e-9;
const double pertMagnitude = 1e-6;

double wi_stddev_ = 0.0035;
double ai_stddev_ = 0.0157;
double bg_stddev_ = 0.0005;
double ba_stddev_ = 0.0005;
double gravityMagnitude_ = -9.80665;

bool printTests = false;
int numTests = 100;

class ImuTestState : public State {
 public:
  ImuTestState(bool optGravity, double t = 0.0) : State(t, 1, 1) {
    processChains_[0] = std::make_shared<ImuChain>(optGravity);

    parameters_.emplace_back(confusion::Parameter(T_w_i_.trans.data(), 3, "t_w_i"));
    parameters_.emplace_back(confusion::Parameter(T_w_i_.rot.coeffs().data(),
                                                  4,
                                                  "q_w_i",
                                                  false,
                                                  std::make_shared<QuatParam>()));
    parameters_.emplace_back(confusion::Parameter(linVel_.data(), 3, "linVel"));
    parameters_.emplace_back(confusion::Parameter(accelBias_.data(), 3, "accelBias"));
    parameters_.emplace_back(confusion::Parameter(gyroBias_.data(), 3, "gyroBias"));
  }

  bool initFirstState(
      const std::vector<std::deque<std::shared_ptr<ProcessMeasurement>>> &processMeasBuffer,
      const std::vector<std::deque<std::shared_ptr<UpdateMeasurement>>> &updateMeasBuffer,
      StaticParameterVector &staticParameters) {
    return true;
  }

  std::shared_ptr<State> createNextState(
      const std::vector<std::deque<std::shared_ptr<ProcessMeasurement>>> &processMeasBuffer,
      const std::vector<std::deque<std::shared_ptr<UpdateMeasurement>>> &updateMeasBuffer,
      StaticParameterVector &staticParameters) {
    auto state = std::make_shared<ImuTestState>(this);
    return state;
  }

  Pose<double> T_w_i_;
  Eigen::Vector3d linVel_;
  Eigen::Vector3d accelBias_;
  Eigen::Vector3d gyroBias_;
};

void check_vec(Eigen::VectorXd v1, Eigen::VectorXd v2, Eigen::VectorXd dv_est, bool print = false) {
  //Print results
  Eigen::VectorXd v2_est = v1 + dv_est;
  Eigen::VectorXd dv = v2 - v1;
//	std::cout << "test: " << (dv-dv_est).transpose() << std::endl;

  if (print) {
//		std::cout << "v1 =[" << v1.transpose() << "]" << std::endl;
//		std::cout << "v2 =[" << v2.transpose() << "]" << std::endl;
//		std::cout << "v2e=[" << v2_est.transpose() << "]\n" << std::endl;

    std::cout << "dv_in =[" << dv.transpose() << "]" << std::endl;
    std::cout << "dv_est =[" << dv_est.transpose() << "]" << std::endl;
    std::cout << "error=[" << (dv - dv_est).transpose() << "]" << std::endl;
  }

  for (int i = 0; i < v1.size(); ++i) {
    EXPECT_LT(fabs(dv(i) - dv_est(i)), tolerance);
  }
}

//NOTE: This doesn't pass unless S is set to identity in the imu_cost evaluation
TEST(imu_cost_jacob_test, ShouldPass) {

  StaticParameterVector staticParameterVector;
  Eigen::Vector2d gwr(Eigen::Vector2d::Random());
  staticParameterVector.addParameter(confusion::Parameter(gwr.data(), 2, "g_r"));

  bool optGravitySettings[] = {true, false};
  for (auto optGravity: optGravitySettings) {
    for (int i = 0; i < numTests; ++i) {
      std::cout.precision(8);

      //Test the imu error term jacobians
      Eigen::Vector3d p0(Eigen::Vector3d::Random());
      Eigen::Vector3d v0(Eigen::Vector3d::Random());
      Eigen::Quaterniond q0(Eigen::Vector4d::Random());
      q0.normalize();
      Eigen::Vector3d ba0(Eigen::Vector3d::Random());
      ba0 *= 0.1;
      Eigen::Vector3d bg0(Eigen::Vector3d::Random());
      bg0 *= 0.1;

      auto state0 = std::make_shared<ImuTestState>(optGravity, 0.005);
      state0->T_w_i_.trans = p0;
      state0->T_w_i_.rot = q0;
      state0->linVel_ = v0;
      state0->accelBias_ = ba0;
      state0->gyroBias_ = bg0;

      Eigen::Vector3d p1 = p0;
      Eigen::Vector3d v1 = v0;
      Eigen::Quaterniond q1 = q0;
      Eigen::Vector3d ba1 = ba0;
      Eigen::Vector3d bg1 = bg0;

      auto state1 = std::make_shared<ImuTestState>(optGravity, 0.045);
      state1->T_w_i_.trans = p0;
      state1->T_w_i_.rot = q0;
      state1->linVel_ = v0;
      state1->accelBias_ = ba0;
      state1->gyroBias_ = bg0;

      gwr.setRandom();

      ImuCalibration imuCalibration;
      imuCalibration.cov_imu_nominal_.setIdentity();
      imuCalibration.cov_imu_nominal_.block<3, 3>(0, 0) *= wi_stddev_ * wi_stddev_;
      imuCalibration.cov_imu_nominal_.block<3, 3>(3, 3) *= ai_stddev_ * ai_stddev_;
      imuCalibration.cov_imu_nominal_.block<3, 3>(6, 6) *= bg_stddev_ * bg_stddev_;
      imuCalibration.cov_imu_nominal_.block<3, 3>(9, 9) *= ba_stddev_ * ba_stddev_;
      imuCalibration.gravityMagnitude_ = gravityMagnitude_;
      imuCalibration.g_w_ << 0.0, 0.0, gravityMagnitude_;

      Eigen::Vector3d wi(Eigen::Vector3d::Random());
      Eigen::Vector3d ai(Eigen::Vector3d::Random());
      auto meas1 = std::make_shared<ImuMeas>(0.0, ai, wi, &imuCalibration, &gwr);
      auto meas2 = std::make_shared<ImuMeas>(0.01, ai, wi, &imuCalibration, &gwr);
      auto meas3 = std::make_shared<ImuMeas>(0.02, ai, wi, &imuCalibration, &gwr);
      auto meas4 = std::make_shared<ImuMeas>(0.03, ai, wi, &imuCalibration, &gwr);
      auto meas5 = std::make_shared<ImuMeas>(0.04, ai, wi, &imuCalibration, &gwr);

      ImuChain imuChain(optGravity);
      imuChain.measurements_.push_back(meas1);
      imuChain.measurements_.push_back(meas2);
      imuChain.measurements_.push_back(meas3);
      imuChain.measurements_.push_back(meas4);
      imuChain.measurements_.push_back(meas5);
      imuChain.assignTimes(state0->t(), state1->t());

      StaticParameterVector staticParameterVector;
      confusion::Parameter gravityRotParam(gwr.data(), 2, "g_rot");
      staticParameterVector.addParameter(gravityRotParam);

      std::unique_ptr<ceres::CostFunction> costFunctionPtr;
      std::unique_ptr<ceres::LossFunction> lossFunctionPtr;
      std::vector<size_t> stateParameterIndexVector;
      std::vector<double *> staticParameterDataVector;
      EXPECT_TRUE(imuChain.createCostFunction(costFunctionPtr,
                                              lossFunctionPtr,
                                              stateParameterIndexVector,
                                              staticParameterDataVector));

      EXPECT_EQ(stateParameterIndexVector.size(), 5);
      if (optGravity)
        EXPECT_EQ(staticParameterDataVector.size(), 1);
      else
        EXPECT_EQ(staticParameterDataVector.size(), 0);

      std::vector<double *> parameterDataVector;
      for (auto &index: stateParameterIndexVector) {
        parameterDataVector.push_back(state0->parameters_[index].data_);
      }
      for (auto &index: stateParameterIndexVector) {
        parameterDataVector.push_back(state1->parameters_[index].data_);
      }
      for (auto &dataPtr: staticParameterDataVector) {
        Parameter *parameter = staticParameterVector.getParameter(dataPtr);
        EXPECT_TRUE(parameter);
        parameterDataVector.push_back(dataPtr);
      }

      Eigen::Matrix<double, 15, 3, Eigen::RowMajor> de_dp0(Eigen::Matrix<double, 15, 3, Eigen::RowMajor>::Zero());
      Eigen::Matrix<double, 15, 4, Eigen::RowMajor> de_dq0(Eigen::Matrix<double, 15, 4, Eigen::RowMajor>::Zero());
      Eigen::Matrix<double, 15, 3, Eigen::RowMajor> de_dv0(Eigen::Matrix<double, 15, 3, Eigen::RowMajor>::Zero());
      Eigen::Matrix<double, 15, 3, Eigen::RowMajor> de_dba0(Eigen::Matrix<double, 15, 3, Eigen::RowMajor>::Zero());
      Eigen::Matrix<double, 15, 3, Eigen::RowMajor> de_dbg0(Eigen::Matrix<double, 15, 3, Eigen::RowMajor>::Zero());
      Eigen::Matrix<double, 15, 3, Eigen::RowMajor> de_dp1(Eigen::Matrix<double, 15, 3, Eigen::RowMajor>::Zero());
      Eigen::Matrix<double, 15, 4, Eigen::RowMajor> de_dq1(Eigen::Matrix<double, 15, 4, Eigen::RowMajor>::Zero());
      Eigen::Matrix<double, 15, 3, Eigen::RowMajor> de_dv1(Eigen::Matrix<double, 15, 3, Eigen::RowMajor>::Zero());
      Eigen::Matrix<double, 15, 3, Eigen::RowMajor> de_dba1(Eigen::Matrix<double, 15, 3, Eigen::RowMajor>::Zero());
      Eigen::Matrix<double, 15, 3, Eigen::RowMajor> de_dbg1(Eigen::Matrix<double, 15, 3, Eigen::RowMajor>::Zero());
      Eigen::Matrix<double, 15, 2, Eigen::RowMajor> de_dgwr(Eigen::Matrix<double, 15, 2, Eigen::RowMajor>::Zero());

      std::vector<double *> jacobians;
      jacobians.push_back(de_dp0.data());
      jacobians.push_back(de_dq0.data());
      jacobians.push_back(de_dv0.data());
      jacobians.push_back(de_dba0.data());
      jacobians.push_back(de_dbg0.data());
      jacobians.push_back(de_dp1.data());
      jacobians.push_back(de_dq1.data());
      jacobians.push_back(de_dv1.data());
      jacobians.push_back(de_dba1.data());
      jacobians.push_back(de_dbg1.data());
      if (optGravity)
        jacobians.push_back(de_dgwr.data());

      Eigen::Matrix<double, 15, 1> e;
      costFunctionPtr->Evaluate(parameterDataVector.data(), e.data(), jacobians.data());

      Eigen::Matrix<double, 15, 1> de_est;
      Eigen::Matrix<double, 15, 1> e_p;

      //Check de_p0
      Eigen::Vector3d dp0(Eigen::Vector3d::Random());
      dp0 *= pertMagnitude;
      Eigen::Vector3d p0_p = p0 + dp0;
      parameterDataVector[0] = p0_p.data();

      costFunctionPtr->Evaluate(parameterDataVector.data(), e_p.data(), nullptr); //jacobians_p.data());

      de_est = de_dp0 * dp0;
      check_vec(e, e_p, de_est, printTests);
      parameterDataVector[0] = p0.data();


      //Check de_q0
      Eigen::Vector4d dq0(Eigen::Vector4d::Random());
      dq0 *= pertMagnitude;
      //Order dq [x,y,z,w] to match Eigen's underlying data structure, which is also the order of the returned Jacobian
      Eigen::Quaterniond q0_p(q0.w() + dq0(3), q0.x() + dq0(0), q0.y() + dq0(1), q0.z() + dq0(2));
      parameterDataVector[1] = q0_p.coeffs().data();

      costFunctionPtr->Evaluate(parameterDataVector.data(), e_p.data(), nullptr); //jacobians_p.data());

      de_est = de_dq0 * dq0;
      check_vec(e, e_p, de_est, printTests);
      parameterDataVector[1] = q0.coeffs().data();


      //Check de_v0
      Eigen::Vector3d dv0(Eigen::Vector3d::Random());
      dv0 *= pertMagnitude;
      Eigen::Vector3d v0_p = v0 + dv0;
      parameterDataVector[2] = v0_p.data();

      costFunctionPtr->Evaluate(parameterDataVector.data(), e_p.data(), nullptr); //jacobians_p.data());

      de_est = de_dv0 * dv0;
      check_vec(e, e_p, de_est, printTests);
      parameterDataVector[2] = v0.data();


      //Check de_ba0
      Eigen::Vector3d dba0(Eigen::Vector3d::Random());
      dba0 *= pertMagnitude;
      Eigen::Vector3d ba0_p = ba0 + dba0;
      parameterDataVector[3] = ba0_p.data();

      costFunctionPtr->Evaluate(parameterDataVector.data(), e_p.data(), nullptr); //jacobians_p.data());

      de_est = de_dba0 * dba0;
      check_vec(e, e_p, de_est, printTests);
      parameterDataVector[3] = ba0.data();


      //Check de_bg0
      Eigen::Vector3d dbg0(Eigen::Vector3d::Random());
      dbg0 *= pertMagnitude;
      Eigen::Vector3d bg0_p = bg0 + dbg0;
      parameterDataVector[4] = bg0_p.data();

      costFunctionPtr->Evaluate(parameterDataVector.data(), e_p.data(), nullptr); //jacobians_p.data());

      de_est = de_dbg0 * dbg0;
      check_vec(e, e_p, de_est, printTests);
      parameterDataVector[4] = bg0.data();


      //Check de_p1
      Eigen::Vector3d dp1(Eigen::Vector3d::Random());
      dp1 *= pertMagnitude;
      Eigen::Vector3d p1_p = p1 + dp1;
      parameterDataVector[5] = p1_p.data();

      costFunctionPtr->Evaluate(parameterDataVector.data(), e_p.data(), nullptr); //jacobians_p.data());

      de_est = de_dp1 * dp1;
      check_vec(e, e_p, de_est, printTests);

      parameterDataVector[5] = p1.data();


      //Check de_q1
      Eigen::Vector4d dq1(Eigen::Vector4d::Random());
      dq1 *= pertMagnitude;
      //Order dq [x,y,z,w] to match Eigen's underlying data structure, which is also the order of the returned Jacobian
      Eigen::Quaterniond q1_p(q1.w() + dq1(3), q1.x() + dq1(0), q1.y() + dq1(1), q1.z() + dq1(2));
      parameterDataVector[6] = q1_p.coeffs().data();

      costFunctionPtr->Evaluate(parameterDataVector.data(), e_p.data(), nullptr); //jacobians_p.data());

      de_est = de_dq1 * dq1;
      check_vec(e, e_p, de_est, printTests);
      parameterDataVector[6] = q1.coeffs().data();

      //Check de_v1
      Eigen::Vector3d dv1(Eigen::Vector3d::Random());
      dv1 *= pertMagnitude;
      Eigen::Vector3d v1_p = v1 + dv1;
      parameterDataVector[7] = v1_p.data();

      costFunctionPtr->Evaluate(parameterDataVector.data(), e_p.data(), nullptr); //jacobians_p.data());

      de_est = de_dv1 * dv1;
      check_vec(e, e_p, de_est, printTests);
      parameterDataVector[7] = v1.data();


      //Check de_ba1
      Eigen::Vector3d dba1(Eigen::Vector3d::Random());
      dba1 *= pertMagnitude;
      Eigen::Vector3d ba1_p = ba1 + dba1;
      parameterDataVector[8] = ba1_p.data();

      costFunctionPtr->Evaluate(parameterDataVector.data(), e_p.data(), nullptr); //jacobians_p.data());

      de_est = de_dba1 * dba1;
      check_vec(e, e_p, de_est, printTests);
      parameterDataVector[8] = ba1.data();


      //Check de_bg1
      Eigen::Vector3d dbg1(Eigen::Vector3d::Random());
      dbg1 *= pertMagnitude;
      Eigen::Vector3d bg1_p = bg1 + dbg1;
      parameterDataVector[9] = bg1_p.data();

      costFunctionPtr->Evaluate(parameterDataVector.data(), e_p.data(), nullptr); //jacobians_p.data());

      de_est = de_dbg1 * dbg1;
      check_vec(e, e_p, de_est, printTests);
      parameterDataVector[9] = bg1.data();


      //Check de_dgw
      if (optGravity) {
        Eigen::Vector2d dgwr(Eigen::Vector2d::Random());
        dgwr *= 0.00001;
        Eigen::Vector2d gwr_p = gwr + dgwr;
        parameterDataVector[10] = gwr_p.data();

        costFunctionPtr->Evaluate(parameterDataVector.data(), e_p.data(), nullptr); //jacobians_p.data());

        de_est = de_dgwr * dgwr;
        check_vec(e, e_p, de_est, printTests);
        parameterDataVector[10] = gwr.data();
      }
    }
  }
}

int main(int argc, char **argv) {
  srand(time(NULL));
  std::cout.precision(12);

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
