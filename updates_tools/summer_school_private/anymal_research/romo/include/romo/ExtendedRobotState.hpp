/*
 * ExtendedRobotState.hpp
 *
 *  Created on: Dez 8, 2016
 *      Author: Dominic Jud
 */

#pragma once

// robot state
#include "romo/common/RobotDescription.hpp"
#include "romo/RobotState.hpp"

namespace romo {

template <typename ConcreteDescription_>
class ExtendedRobotState : public RobotState<ConcreteDescription_> {

  using Base = RobotState<ConcreteDescription_>;

 protected:
  using RD = RobotDescription<ConcreteDescription_>;

 public:
  using GeneralizedCoordinates =    typename Base::GeneralizedCoordinates;
  using GeneralizedVelocities =     typename Base::GeneralizedVelocities;
  using GeneralizedAccelerations =  typename Base::GeneralizedAccelerations;
  using JointPositions =            typename Base::JointPositions;
  using JointVelocities =           typename Base::JointVelocities;
  using JointAccelerations =        typename Base::JointAccelerations;
  using JointTorques =              typename Base::JointTorques;

 public:
  ExtendedRobotState() = default;
  virtual ~ExtendedRobotState() = default;

  const JointAccelerations& getJointAccelerations() const;
  JointAccelerations& getJointAccelerations();
  const JointTorques& getJointTorques() const;
  JointTorques& getJointTorques();
  const LinearAcceleration& getLinearAccelerationBaseInWorldFrame() const;
  LinearAcceleration& getLinearAccelerationBaseInWorldFrame();
  const AngularAcceleration& getAngularAccelerationBaseInBaseFrame() const;
  AngularAcceleration& getAngularAccelerationBaseInBaseFrame();

  void setJointAccelerations(const JointAccelerations& JointAccelerations);
  void setJointTorques(const JointTorques& jointTorques);
  void setLinearAccelerationBaseInWorldFrame(const LinearAcceleration& linearAccelerationBaseInWorldFrame);
  void setAngularAccelerationBaseInBaseFrame(const AngularAcceleration& angularAccelerationBaseInBaseFrame);

  void setRandom() override;

  /*! Sets all positions and velocities to zero and rotations to the identity.
   */
  void setZero() override;

 protected:
  JointTorques jointTorques_;
  JointAccelerations jointAccelerations_;
  LinearAcceleration linearAccelerationBaseInWorldFrame_;
  AngularAcceleration angularAccelerationBaseInBaseFrame_;
};

}

#include <romo/ExtendedRobotState.tpp>
