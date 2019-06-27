//
// Created by tim on 28.01.19.
//

#ifndef SMB_SMBPROPAGATOR_H
#define SMB_SMBPROPAGATOR_H

#include <ceres/ceres.h>
#include <ros/ros.h>
#include "smb_confusor/SmbState.h"
#include "confusion/ProcessChain.h"

namespace smb_confusor {

class SmbPropagator {
 public:
  SmbPropagator();

  void addProcessMeasurement(std::shared_ptr<confusion::ProcessMeasurement> processMeasurement);

  void setEstimate(const confusion::ImuStateParameters &stateParams);

  bool getPropagatedEstimate(const double &t, confusion::ImuStateParameters &stateParamsOut);

  /**
   * Right now, this will just change the initial value of the propagated state for the next query
   */
  void reset() { ready_ = false; }

 private:

  void buildProblem();

  bool ready_ = false; //Set true once the first estimate is received
  SmbState lastEstimate_;
  SmbState propagatedEstimate_;

  ceres::Problem::Options problemOptions_;
  ceres::Solver::Summary summary_;
  ceres::Solver::Options solverOptions_;
  std::unique_ptr<ceres::Problem> problem_;
  confusion::StaticParameterVector staticParameters_;
  int stateInitializationSensor_ = IMU;
};

} // namespace smb_confusor

#endif //SMB_SMBPROPAGATOR_H
