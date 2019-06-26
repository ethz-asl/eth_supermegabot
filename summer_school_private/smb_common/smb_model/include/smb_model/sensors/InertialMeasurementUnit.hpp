/*!
 * @file 	InertialMeasurementUnit.hpp
 * @author 	Christian Gehring
 * @date 	  Dec 2014
 * @version 1.0
 * @ingroup robot_model
 */

#pragma once

#include <Eigen/Core>

namespace quadruped_model {

class InertialMeasurementUnit {
  friend class Sensors;
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  InertialMeasurementUnit() :
    accWhiNoiVar_(0.0),
    accStd_(0.0),
    gyrWhiNoiVar_(0.0),
    gyrStd_(0.0)
  {

  };

  virtual ~InertialMeasurementUnit(){};

  Eigen::Vector3d getAccelerometerData() {
    return imuDataAcc_;
  }

  void setAccelerometerData(const Eigen::Vector3d& imuDataAcc)  {
    imuDataAcc_ = imuDataAcc;
  }

  Eigen::Vector3d getGyrometerData() {
    return imuDataGyr_;
  }

  void setGyrometerData(const Eigen::Vector3d& imuDataGyr) {
    imuDataGyr_ = imuDataGyr;
  }


protected:
  //! Accelerometer measurements (expressed in base frame)
  Eigen::Vector3d imuDataAcc_;

  //! Simulated accelerometer bias
  Eigen::Vector3d accBias_;

  //! Gyrometer measurements
  Eigen::Vector3d imuDataGyr_;

  //! Simulated Gyrometer bias  (expressed in base frame)
  Eigen::Vector3d gyrBias_;

  //! Accelerometer white noisesvariance parameter
  double accWhiNoiVar_;

  //! Standard deviation of Accelerometer measurements
  double accStd_;

  //! Gyrometer white noise variance parameter
  double gyrWhiNoiVar_;

  //! Standard deviation of gyrometer measurements
  double gyrStd_;

  //! Gravitational acceleration
  Eigen::Vector3d I_g_;
};

} // namespace quadruped_model
