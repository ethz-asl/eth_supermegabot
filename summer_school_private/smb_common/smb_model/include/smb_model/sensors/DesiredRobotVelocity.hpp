/*!
 * @file 	DesiredRobotVelocity.hpp
 * @author 	Christian Gehring
 * @date 	  Dec 2014
 * @version 1.0
 * @ingroup robot_model
 */

#pragma once

#include <Eigen/Core>

namespace quadruped_model {

class DesiredRobotVelocity {
  friend class Sensors;
public:
  DesiredRobotVelocity():desSagittalVelocity_(0.0), desCoronalVelocity_(0.0), desTurningRate_(0.0)
  {

  }
  virtual ~DesiredRobotVelocity() {

  }
  double getDesSagittalVelocity() {
    return desSagittalVelocity_;
  }

  double getDesCoronalVelocity() {
    return desCoronalVelocity_;
  }

  double getDesTurningRate() {
    return desTurningRate_;
  }

  Eigen::Vector3d getDesVelocities() {
    return Eigen::Vector3d(desSagittalVelocity_, desCoronalVelocity_, desTurningRate_);
  }

  void setDesSagittalVelocity(double velocity) {
    desSagittalVelocity_ = velocity;
  }

  void setDesCoronalVelocity(double velocity) {
    desCoronalVelocity_ = velocity;
  }

  void setDesTurningRate(double velocity) {
    desTurningRate_ = velocity;
  }

  void setDesVelocities(const Eigen::Vector3d& velocities) {
    desSagittalVelocity_ = velocities(0);
    desCoronalVelocity_ = velocities(1);
    desTurningRate_ = velocities(2);
  }

protected:
  double desSagittalVelocity_;
  double desCoronalVelocity_;
  double desTurningRate_;
};

} // namespace quadruped_model
