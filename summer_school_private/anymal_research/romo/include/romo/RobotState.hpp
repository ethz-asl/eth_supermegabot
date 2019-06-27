/*
 * RobotState.hpp
 *
 *  Created on: Nov 10, 2015
 *      Author: Dario Bellicoso
 */

#pragma once

// state interface
#include "romo/common/phys_typedefs.hpp"
#include "romo/common/robot_state_typedefs.hpp"
#include "romo/common/RobotDescription.hpp"

namespace romo {

template <typename ConcreteDescription_>
class RobotState {

 protected:
  using RD = RobotDescription<ConcreteDescription_>;

 public:
  using GeneralizedCoordinates =    typename romo::internal::GeneralizedCoordinates<RD::getGeneralizedCoordinatesDimension()>;
  using GeneralizedVelocities =     typename romo::internal::GeneralizedVelocities<RD::getGeneralizedVelocitiesDimension()>;
  using GeneralizedAccelerations =  typename romo::internal::GeneralizedAccelerations<RD::getGeneralizedAccelerationsDimension()>;
  using JointPositions =            typename romo::internal::JointPositions<RD::getJointsDimension()>;
  using JointVelocities =           typename romo::internal::JointVelocities<RD::getJointsDimension()>;
  using JointAccelerations =        typename romo::internal::JointAccelerations<RD::getJointsDimension()>;
  using JointTorques =              typename romo::internal::JointTorques<RD::getJointsDimension()>;

 public:
  RobotState() = default;
  virtual ~RobotState() = default;

  //static getter methods for system dimensions (template arguments)
  static constexpr unsigned int getNumberOfGeneralizedCoordinates()   { return RD::getGeneralizedCoordinatesDimension(); }
  static constexpr unsigned int getNumberOfGeneralizedVelocities()    { return RD::getGeneralizedVelocitiesDimension();   }
  static constexpr unsigned int getNumberOfGeneralizedAccelerations() { return RD::getGeneralizedAccelerationsDimension();   }
  static constexpr unsigned int getNumberOfJointPositions()           { return RD::getJointsDimension();  }
  static constexpr unsigned int getNumberOfJointVelocities()          { return RD::getJointsDimension();  }
  static constexpr unsigned int getNumberOfJointTorques()             { return RD::getJointsDimension();  }

  GeneralizedCoordinates getGeneralizedCoordinates() const;
  GeneralizedVelocities getGeneralizedVelocities() const;

  const Position& getPositionWorldToBaseInWorldFrame() const;
  const RotationQuaternion& getOrientationBaseToWorld() const;

  const LinearVelocity& getLinearVelocityBaseInWorldFrame() const;
  const LocalAngularVelocity& getAngularVelocityBaseInBaseFrame() const;

  const JointPositions& getJointPositions() const;
  JointPositions& getJointPositions();
  const JointVelocities& getJointVelocities() const;
  JointVelocities& getJointVelocities();

  void setPositionWorldToBaseInWorldFrame(const Position& positionWorldToBaseInWorldFrame);
  void setOrientationBaseToWorld(const RotationQuaternion& orientationBaseToWorld);
  void setLinearVelocityBaseInWorldFrame(const LinearVelocity& linearVelocityBaseInWorldFrame);
  void setAngularVelocityBaseInBaseFrame(const LocalAngularVelocity& angularVelocityBaseInBaseFrame);

  void setJointPositions(const JointPositions& jointPositions);
  void setJointVelocities(const JointVelocities& jointVelocities);

  virtual void setRandom();

  /*! Sets all positions and velocities to zero and rotations to the identity.
   */
  virtual void setZero();

  virtual void setZeroVelocities();

 protected:
  Position positionWorldToBaseInWorldFrame_;
  RotationQuaternion orientationBaseToWorld_;
  JointPositions jointPositions_;

  LinearVelocity linearVelocityBaseInWorldFrame_;
  LocalAngularVelocity angularVelocityBaseInBaseFrame_;
  JointVelocities jointVelocities_;
};

}

#include <romo/RobotState.tpp>
