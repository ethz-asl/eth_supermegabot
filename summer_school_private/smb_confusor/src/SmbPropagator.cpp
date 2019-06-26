//
// Created by tim on 28.01.19.
//

#include "smb_confusor/SmbPropagator.h"

namespace smb_confusor {

SmbPropagator::SmbPropagator() :
    lastEstimate_(stateInitializationSensor_),
    propagatedEstimate_(stateInitializationSensor_) {
  //todo Make these configurable
  solverOptions_.max_num_iterations = 5;
  solverOptions_.num_threads = 1;

  problemOptions_.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  problemOptions_.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  problemOptions_.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;

  //Set the parameters of the leading state constant
  for (auto &param: lastEstimate_.parameters_)
    param.setConstant();
}

void SmbPropagator::addProcessMeasurement(std::shared_ptr<confusion::ProcessMeasurement> measPtr) {
  if (measPtr->measType() < lastEstimate_.numProcessSensors()) {
    lastEstimate_.processChains_[measPtr->measType()]->measurements_.push_back(measPtr);
  }
  else {
    std::cout << "ERROR: SmbPropagator received a process measurement of unknown type!?" << std::endl;
  }
}

void SmbPropagator::setEstimate(const confusion::ImuStateParameters &stateParams) {
  //Set the state parameters to the estimate
  lastEstimate_.t_ = stateParams.t_;
  lastEstimate_.T_w_i_ = stateParams.T_w_i_;
  lastEstimate_.linVel_ = stateParams.linVel_;
  lastEstimate_.accelBias_ = stateParams.accelBias_;
  lastEstimate_.gyroBias_ = stateParams.gyroBias_;

  if (!ready_) {
    //Also "warm-start" the propagated state parameters to the same if this is the first estimate update
    propagatedEstimate_.t_ = stateParams.t_;
    propagatedEstimate_.T_w_i_ = stateParams.T_w_i_;
    propagatedEstimate_.linVel_ = stateParams.linVel_;
    propagatedEstimate_.accelBias_ = stateParams.accelBias_;
    propagatedEstimate_.gyroBias_ = stateParams.gyroBias_;

    buildProblem();

    ready_ = true;
  }

  //Remove any old process measurements from the measurement buffers
  for (int j=0; j<lastEstimate_.numProcessSensors(); ++j) {
    auto iter = lastEstimate_.processChains_[j]->measurements_.begin();
    while (iter != lastEstimate_.processChains_[j]->measurements_.end() && (*iter)->t() < stateParams.t_) {
      ++iter;
    }

    if (iter == lastEstimate_.processChains_[j]->measurements_.end())
      std::cout << "WARNING: SmbPropagator::setEstimate called with the state later than the most recent "
                   "process measurements of type " << j << std::endl;

    if (iter != lastEstimate_.processChains_[j]->measurements_.begin()) {
      //We actually want the measurement at or before the set time
      if ((*iter)->t() > stateParams.t_)
        --iter;

      //Erase the old measurements
      lastEstimate_.processChains_[j]->measurements_.erase(lastEstimate_.processChains_[j]->measurements_.begin(),iter);
    }
  }
}

bool SmbPropagator::getPropagatedEstimate(const double &t, confusion::ImuStateParameters &stateParamsOut) {
  ros::Time t0 = ros::Time::now();

  if (!ready_)
    return false;

  propagatedEstimate_.t_ = t;

  //Set the new termination time for the process chains
  for (int j=0; j<lastEstimate_.numProcessSensors(); ++j) {
    lastEstimate_.processChains_[j]->assignTimes(lastEstimate_.t(), t);
  }

  //Solve
  ceres::Solve(solverOptions_, problem_.get(), &summary_);

  ros::Time t1 = ros::Time::now();

  std::cout << summary_.BriefReport() << std::endl;
  std::cout << "Optimization has " << summary_.num_parameter_blocks_reduced << " parameters active" << std::endl;
  std::cout << "SmbPropagator::getPropagatedEstimate for dt=" <<
      propagatedEstimate_.t() - lastEstimate_.t() << " took " << (t1-t0).toSec() << std::endl;

  propagatedEstimate_.getAngularVelocity(); //This sets the angular velocity from the closest IMU measurement
  stateParamsOut = propagatedEstimate_.getParameterStruct();

  return true;
}

void SmbPropagator::buildProblem() {
  problem_ = std::unique_ptr<ceres::Problem>(new ceres::Problem(problemOptions_));

  //Add the process costs
  for (int j=0; j<lastEstimate_.numProcessSensors(); ++j) {
    if (!lastEstimate_.processChains_[j]->addCostToProblem(problem_.get(),
                                                           lastEstimate_.parameters_,
                                                           propagatedEstimate_.parameters_,
                                                           staticParameters_)) {
      std::cout << "SmbPropagator -- Failed to add process cost type " << j << " to the problem" << std::endl;
    }
  }

  //Add the state parameters to the problem
  for (auto param: lastEstimate_.parameters_)
    param.addToProblem(problem_.get());
  for (auto param: propagatedEstimate_.parameters_)
    param.addToProblem(problem_.get());

  //todo There are no static parameters for now (opt_gravity is off)
}

} // namespace smb_confusor