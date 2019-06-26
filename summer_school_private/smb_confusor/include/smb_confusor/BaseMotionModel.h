/*
 * BaseMotionModel.h
 *
 *  Created on: Feb 19, 2018
 *      Author: tim
 */

#ifndef INCLUDE_SMB_STATE_ESTIMATOR_BASEMOTIONMODEL_H_
#define INCLUDE_SMB_STATE_ESTIMATOR_BASEMOTIONMODEL_H_

#include <confusion/utilities/Pose.h>
#include "confusion/utilities/rotation_utils.h"
#include "confusion/utilities/ceres_utils.h"
#include "confusion/utilities/distances.h"
#include "confusion/ProcessChain.h"

struct BaseMotionModelParameters {
  double t;
  confusion::Pose<double> T_w_b;
};

struct BaseMotionModelCalibration {
  bool useLossFunction_ = false;
  double lossCoefficient_ = 0.0;
  Eigen::Matrix<double, 6, 6> cov_Twb_stationary; //todo Need to set these in the constructor??
  Eigen::Matrix<double, 6, 6> cov_Twb_moving;
  double movingWheelSpeedThld_ = 0.01; //[rad/sec] Consider the base as moving when one wheel speed is exceeding thos amount
  double wheelRadius_; // [m] Wheel radius
  double wheelbase_; // [m] Distance between the left and right wheel centerlines
};

class WheelSpeeds : public confusion::ProcessMeasurement {
 public:
  WheelSpeeds(double t, double leftWheelSpeed,
              double rightWheelSpeed, BaseMotionModelCalibration &calibration,
              confusion::Pose<double> &T_imu_base, int measurementType) :
      confusion::ProcessMeasurement(measurementType, t), leftWheelSpeed_(leftWheelSpeed),
      rightWheelSpeed_(rightWheelSpeed), calibration_(calibration), T_imu_base_(T_imu_base) {
    double leftWheelForwardVelocity = leftWheelSpeed_ * calibration_.wheelRadius_;
    double rightWheelForwardVelocity = rightWheelSpeed_ * calibration_.wheelRadius_;
    forwardVelocity_ = leftWheelForwardVelocity + rightWheelForwardVelocity / 2.0;
    angularVelocity_ =
        (rightWheelForwardVelocity - leftWheelForwardVelocity) / calibration_.wheelbase_;
//std::cout << "t=" << t << "; leftSpd=" << leftWheelSpeed_ << "; rightSpd=" << rightWheelSpeed_ << "; fwdVel=" << forwardVelocity_ << "; angVel=" << angularVelocity_ << std::endl;
    if (fabs(leftWheelSpeed_) > calibration_.movingWheelSpeedThld_ ||
        fabs(rightWheelSpeed_) > calibration_.movingWheelSpeedThld_) {
      moving_ = true;
    } else
      moving_ = false;
  }

  WheelSpeeds& operator=(const WheelSpeeds& meas) {
    if(this == &meas)
      return *this;

    t_ = meas.t();
    measType_ = meas.measType();
    moving_ = meas.moving_;
    leftWheelSpeed_ = meas.leftWheelSpeed_;
    rightWheelSpeed_ = meas.rightWheelSpeed_;
    calibration_ = meas.calibration_;
    T_imu_base_ = meas.T_imu_base_;
    forwardVelocity_ = meas.forwardVelocity_;
    angularVelocity_ = meas.angularVelocity_;

    return *this;
  }

  bool moving_;
  double leftWheelSpeed_;
  double rightWheelSpeed_;
  BaseMotionModelCalibration &calibration_;
//  confusion::Pose<double> &T_base_imu_;
  confusion::Pose<double> &T_imu_base_;

  double forwardVelocity_;
  double angularVelocity_;
};


template<typename T>
bool forwardIntegrateBaseMotionModel(confusion::Pose<T> &T_w_b,
                                     const WheelSpeeds &wheelSpeeds,
                                     const double &dt,
                                     Eigen::Matrix<double, 6, 6> *cov_Twb = nullptr) {
  if (dt < 0.0) {
    std::cout << "Base motion model integration was given a negative dt! dt=" << dt << std::endl;
    return false;
  }

//  confusion::Pose<double> T_w_b_ = confusion::getDoublesPose(T_w_b);
//    T_w_b_.print("before");

  //Compute differential drive forward kinematics
  Eigen::Matrix<T,3,1> linearVelocity_b;
  linearVelocity_b << T(wheelSpeeds.forwardVelocity_), T(0.0), T(0.0);
  T_w_b.trans += T_w_b.rot * linearVelocity_b * T(dt);
  Eigen::Matrix<T,3,1> angularVelocity_b;
  angularVelocity_b << T(0.0), T(0.0), T(wheelSpeeds.angularVelocity_);
  Eigen::Matrix<T,3,1> drot_w = T_w_b.rot * angularVelocity_b * T(dt);
  T_w_b.rot = confusion::quaternionBoxPlus(T_w_b.rot, drot_w);

  if (cov_Twb) {
    Eigen::Matrix<double, 6, 6> d_cov_b;
    if (wheelSpeeds.moving_)
      d_cov_b = wheelSpeeds.calibration_.cov_Twb_moving;
    else
      d_cov_b = wheelSpeeds.calibration_.cov_Twb_stationary;

    //Rotate the covariance to transform it to the world coordinate frame
    Eigen::Quaterniond q_w_b;
    confusion::getDoubles(T_w_b.rot.coeffs().data(), 4, q_w_b.coeffs().data());

    Eigen::Matrix3d R_w_b = q_w_b.toRotationMatrix();
    Eigen::Matrix<double, 6, 6> d_cov_w(Eigen::Matrix<double, 6, 6>::Zero());
    d_cov_w.topLeftCorner(3, 3) = R_w_b * d_cov_b.topLeftCorner(3, 3);
    d_cov_w.bottomRightCorner(3, 3) = R_w_b * d_cov_b.bottomRightCorner(3, 3);
//    std::cout << "R_w_b:\n" << R_w_b << "\ncov_b:\n" << d_cov_b.topLeftCorner(3, 3) << "\ncov_w:\n" << d_cov_w.topLeftCorner(3, 3) << std::endl;

    (*cov_Twb) += d_cov_w * dt;
  }

//  T_w_b_ = confusion::getDoublesPose(T_w_b);
//    T_w_b_.print("after");

  return true;
}

void forwardPropagateBmm(const std::deque<std::shared_ptr<confusion::ProcessMeasurement>>& bmmMeasurements,
                         const double& t_in, const double& t_des, confusion::Pose<double> &T_w_b) {
  //Do some timing checks
  if (bmmMeasurements.front()->t()-t_in > 0.01) {
    std::cout << "forwardPropagateBmm: The current state is older than the first measurement. t_meas_front-t_state=" <<
                 bmmMeasurements.front()->t() - t_in <<  "; t_meas_back-t_meas_front=" <<
                 bmmMeasurements.back()->t() - bmmMeasurements.front()->t() << std::endl;
  }
  if (t_des > bmmMeasurements.back()->t() + 0.01) {
    std::cout << "forwardPropagateBmm: The last measurement is " << t_des-bmmMeasurements.back()->t() <<
              " sec past the lastest measurement. t_des=" << t_des << "; t_meas_back:" <<
              bmmMeasurements.back()->t() << std::endl;
  }

  std::deque<std::shared_ptr<confusion::ProcessMeasurement>>::const_iterator measIter = bmmMeasurements.begin();

  //Move to the measurement at or before t_in
  while ((measIter + 1) != bmmMeasurements.end() && (*(measIter + 1))->t() <= t_in)
    ++measIter;

  //Propagate from t_in to t_meas_1
  if ((measIter + 1) != bmmMeasurements.end()) {
    if ((*measIter)->t() <= t_in) {
      auto wheelSpeeds = std::dynamic_pointer_cast<WheelSpeeds>(*measIter);
      if (!forwardIntegrateBaseMotionModel(T_w_b, *wheelSpeeds, (*(measIter + 1))->t() - t_in))
        std::cout << "BMM from " << t_in << " to " << t_des << " failed with meas at " << (*(measIter + 1))->t() << std::endl;
      ++measIter;
    } else {
      //This when there is no imu measurement preceeding t_
      auto wheelSpeeds = std::dynamic_pointer_cast<WheelSpeeds>(*measIter);
      if (!forwardIntegrateBaseMotionModel(T_w_b, *wheelSpeeds, (*measIter)->t() - t_in))
        std::cout << "BMM from " << t_in << " to " << t_des << " failed with meas at " << (*(measIter + 1))->t() << std::endl;
    }
  }

  //Iterate through the imu measurements
  while ((measIter + 1) != bmmMeasurements.end() && (*(measIter + 1))->t() <= t_des) {
    auto wheelSpeeds = std::dynamic_pointer_cast<WheelSpeeds>(*measIter);
    if (!forwardIntegrateBaseMotionModel(T_w_b, *wheelSpeeds, (*(measIter + 1))->t() - (*measIter)->t()))
      std::cout << "BMM from " << t_in << " to " << t_des << " failed with meas at " << (*(measIter + 1))->t() << std::endl;
    ++measIter;
  }

  //Propagate the last chunk of time. Don't do anything else if the last measurement is directly at t_des
  if ((*measIter)->t() < t_des) {
    auto wheelSpeeds = std::dynamic_pointer_cast<WheelSpeeds>(*measIter);
    if (!forwardIntegrateBaseMotionModel(T_w_b, *wheelSpeeds, t_des - (*measIter)->t()))
      std::cout << "BMM from " << t_in << " to " << t_des << " failed with meas at " << (*(measIter + 1))->t() << std::endl;
  }
}

class BaseMotionModel;

class BaseMotionCost {
 public:
  BaseMotionCost(BaseMotionModel *baseMotionModel) : baseMotionModel_(baseMotionModel) {}

  template<typename T>
  bool operator()(T const *t_w_ib0, T const *q_w_ib0, //T const* linVel_b0, T const* angVel_b0,
                  T const *t_w_ib1, T const *q_w_ib1, //T const* linVel_b1, T const* angVel_b1,
                  //T const *t_base_imu_, T const *q_base_imu_,
                  T *residual_) const;

  BaseMotionModel *baseMotionModel_;
};

class BaseMotionModel : public confusion::ProcessChain {
 public:
  BaseMotionModel() : confusion::ProcessChain("BMM",false) { }

  ~BaseMotionModel() { }

  void initializeParametersFromMeasurement(const std::shared_ptr<WheelSpeeds> &meas) {
    cov_Twb_stationary_ = meas->calibration_.cov_Twb_stationary;
    cov_Twb_moving_ = meas->calibration_.cov_Twb_moving;
//    T_armBase_b_ = meas->calibration_.T_armBase_b_;
    T_imu_base_ = &(meas->T_imu_base_);
  }

  bool createCostFunction(std::unique_ptr<ceres::CostFunction> &costFunctionPtr,
                          std::unique_ptr<ceres::LossFunction> &lossFunctionPtr,
                          std::vector<size_t> &stateParameterIndexVector,
                          std::vector<double *> &staticParameterDataVector) {
    if (measurements_.empty()) {
      std::cout << "Calling createCostFunction without any measurements in the BaseMotionModel! Unsupported functionality! ti=" <<
                tStart() << ", tj=" << tEnd() << std::endl;
      return false;
    }

    auto firstMeasPtr = std::dynamic_pointer_cast<WheelSpeeds>(measurements_.front());
    initializeParametersFromMeasurement(firstMeasPtr);

    stateParameterIndexVector.push_back(0);
    stateParameterIndexVector.push_back(1);

    if (firstMeasPtr->calibration_.useLossFunction_) {
      std::unique_ptr<ceres::LossFunction> lossFunctionPtr_(new ceres::CauchyLoss(firstMeasPtr->calibration_.lossCoefficient_));
      lossFunctionPtr = std::move(lossFunctionPtr_);
    }
    else
      lossFunctionPtr.reset();

    std::unique_ptr<ceres::CostFunction> constFunctionPtr_(
        //new ceres::AutoDiffCostFunction<BaseMotionCost, 6, 3, 4, 3, 4, 3, 4>(new BaseMotionCost(this)));
        new ceres::AutoDiffCostFunction<BaseMotionCost, 6, 3, 4, 3, 4>(new BaseMotionCost(this)));
    costFunctionPtr = std::move(constFunctionPtr_);

    return true;
  }

  template<typename T>
  bool forwardIntegrate(confusion::Pose<T> &T_w_b, Eigen::Matrix<double, 6, 6> &cov_Twb,
                        const WheelSpeeds &meas, const double &dt) {
    return forwardIntegrateBaseMotionModel<T>(T_w_b, meas, dt, &cov_Twb);
  }

  template<typename T>
  bool compute(confusion::Pose<T> &T_w_b, Eigen::Matrix<double, 6, 6> &cov_Twb) {
    std::deque<std::shared_ptr<confusion::ProcessMeasurement>>::const_iterator measIter = measurements_.begin();

    //Move to the measurement at or before tStart()
    while ((measIter + 1) != measurements_.end() && (*(measIter + 1))->t() <= tStart())
      ++measIter;

    //Propagate from tStart() to t_meas_1
    if ((measIter + 1) != measurements_.end()) {
      if ((*measIter)->t() <= tStart()) {
        auto wheelSpeeds = std::dynamic_pointer_cast<WheelSpeeds>(*measIter);
        if (!forwardIntegrate(T_w_b, cov_Twb, *wheelSpeeds, (*(measIter + 1))->t() - tStart()))
          std::cout << "BMM from " << tStart() << " to " << tEnd() << " failed with meas at " << (*(measIter + 1))->t() << std::endl;
        ++measIter;
      } else {
        //This when there is no imu measurement preceeding t_
        auto wheelSpeeds = std::dynamic_pointer_cast<WheelSpeeds>(*measIter);
        if (!forwardIntegrate(T_w_b, cov_Twb, *wheelSpeeds, (*measIter)->t() - tStart()))
          std::cout << "BMM from " << tStart() << " to " << tEnd() << " failed with meas at " << (*(measIter + 1))->t() << std::endl;
      }
    }

    //Iterate through the imu measurements
    while ((measIter + 1) != measurements_.end() && (*(measIter + 1))->t() <= tEnd()) {
      auto wheelSpeeds = std::dynamic_pointer_cast<WheelSpeeds>(*measIter);
      if (!forwardIntegrate(T_w_b, cov_Twb, *wheelSpeeds, (*(measIter + 1))->t() - (*measIter)->t()))
        std::cout << "BMM from " << tStart() << " to " << tEnd() << " failed with meas at " << (*(measIter + 1))->t() << std::endl;
      ++measIter;
    }

    //Propagate the last chunk of time. Don't do anything else if the last measurement is directly at tEnd()
    if ((*measIter)->t() < tEnd()) {
      auto wheelSpeeds = std::dynamic_pointer_cast<WheelSpeeds>(*measIter);
      if (!forwardIntegrate(T_w_b, cov_Twb, *wheelSpeeds, tEnd() - (*measIter)->t()))
        std::cout << "BMM from " << tStart() << " to " << tEnd() << " failed with meas at " << (*(measIter + 1))->t() << std::endl;
    }

    return true;
  }

  int residualDimension() { return 6; }

  //These get copied in from the first measurement inside of initialize
  confusion::Pose<double> T_armBase_b_;
  Eigen::Matrix<double, 6, 6> cov_Twb_stationary_;
  Eigen::Matrix<double, 6, 6> cov_Twb_moving_;
//  confusion::Pose<double> *T_base_imu_;
  confusion::Pose<double> *T_imu_base_;
};

template<typename T>
bool BaseMotionCost::operator()(T const *t_w_ib0, T const *q_w_ib0, //T const* linVel_b0, T const* angVel_b0,
                                T const *t_w_ib1, T const *q_w_ib1, //T const* linVel_b1, T const* angVel_b1,
                                //T const *t_base_imu_, T const *q_base_imu_,
                                T *residual_) const {
#ifdef COST_DEBUG
  std::cout << "Starting BaseMotionCost" << std::endl;
#endif
  Eigen::Matrix<T, 3, 1> t_w_ib0_(t_w_ib0);
  Eigen::Quaternion<T> q_w_ib0_(q_w_ib0);
  confusion::Pose<T> T_w_ib0(t_w_ib0_, q_w_ib0_);

//  Eigen::Matrix<T, 3, 1> t_base_imu(t_base_imu_);
//  Eigen::Quaternion<T> q_base_imu(q_base_imu_);
//  confusion::Pose<T> T_base_imu(t_base_imu, q_base_imu);

//  confusion::Pose<T> T_w_b = T_w_ib0 * T_base_imu.inverse() * baseMotionModel_->T_armBase_b_;
  confusion::Pose<T> T_imu_base(*baseMotionModel_->T_imu_base_);
  confusion::Pose<T> T_w_b = T_w_ib0 * T_imu_base;

  Eigen::Matrix<double, 6, 6> cov_Twb;
  cov_Twb.setZero();
  baseMotionModel_->compute(T_w_b, cov_Twb);

  Eigen::Matrix<T, 3, 1> t_w_ib1_(t_w_ib1);
  Eigen::Quaternion<T> q_w_ib1_(q_w_ib1);
  confusion::Pose<T> T_w_ib1(t_w_ib1_, q_w_ib1_);
//  confusion::Pose<T> T_w_b1_est = T_w_ib1 * T_base_imu.inverse() * baseMotionModel_->T_armBase_b_;
  confusion::Pose<T> T_w_b1_est = T_w_ib1 * T_imu_base;

  //Compute the error
  Eigen::Map<Eigen::Matrix<T, 6, 1> > e(residual_);
  confusion::QuatDistance(T_w_b.rot, T_w_b1_est.rot, e.data());
  confusion::VectorDistance(T_w_b.trans.data(), T_w_b1_est.trans.data(), e.data() + 3);
//Eigen::Matrix<double,6,1> e_d;
//getDoubles(e.data(), 6, e_d.data());
//std::cout << "e: " << e_d.transpose() << std::endl;
  //Weigh by the stiffness matrix
  Eigen::Matrix<double, 6, 6> inf = cov_Twb.inverse();
  inf = 0.5 * inf + 0.5 * inf.transpose().eval(); //Enforce symmetry
  Eigen::LLT<Eigen::Matrix<double, 6, 6>> llt(inf);
  Eigen::Matrix<double, 6, 6> S = llt.matrixL().transpose();
//std::cout << "S:\n" << S << std::endl;
  e = S.cast<T>() * e;

#ifdef COST_DEBUG
  std::cout << "Done BaseMotionCost" << std::endl;
  Eigen::Matrix<double,6,1> e_;
  confusion::getDoubles(e.data(),6,e_.data());
  std::cout << "BMM Cost: " << e_.transpose() << std::endl;
#endif
  return true;
}

#endif /* INCLUDE_SMB_STATE_ESTIMATOR_BASEMOTIONMODEL_H_ */