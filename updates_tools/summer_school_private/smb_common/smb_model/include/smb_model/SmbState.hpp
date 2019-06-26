/*!
 * @file     SmbState.hpp
 * @author   Koen Kraemer
 * @date     Aug, 2018
 */

#pragma once

// romo
#include <romo/ExtendedRobotState.hpp>

// smb model
#include "smb_model/common/typedefs.hpp"

// smb description
#include "smb_description/SmbDescription.hpp"

// stl
#include <ostream>
#include <unordered_map>

namespace smb_model {

//! Smb State
/*! This class cannot have any dynamically allocated memory, since this object is used in shared memory.
 */
class SmbState : public romo::ExtendedRobotState<smb_description::ConcreteSmbDescription>
{
 private:
  using Base = romo::ExtendedRobotState<smb_description::ConcreteSmbDescription>;

 public:
  using WheelVelocities = typename romo::internal::JointVelocities<RD::ConcreteDefinitions::getNumWheels()>;
  using WheelTorques = typename romo::internal::JointTorques<RD::ConcreteDefinitions::getNumWheels()>;

  SmbState() = default;
  virtual ~SmbState() = default;

  void getPoseBaseToWorld(kindr::HomTransformQuatD& poseBaseToWorld) const;

  void setPoseBaseToWorld(const kindr::HomTransformQuatD& poseBaseToWorld);

  /*! Sets this state equal to the linearly interpolated state between state0 and state1.
   * @param t           interpolation parameter [0, 1]
   * @param state0      state at t=0
   * @param state1      state at t=1
   */
  virtual void setGeneralizedCoordinatesToLinearlyInterpolated(double t, const SmbState& state0, const SmbState& state1);

  /*! @returns this state with an added small delta to a specific state.
   *
   * This operator can be used for finite-difference method.
   *
   * @param delta     value that is added
   * @param uIndex    index of the generalized state.
   */
  virtual SmbState boxPlus(double delta, unsigned int uIndex, bool useQuaternion = true) const;

//TODO necessary (and possible with some of the code I deleted)?
//  const Pose& getFrameTransform(const smb_description::SmbTopology::FrameTransformEnum& transformEnum) const;
//  void setFrameTransform(const smb_description::SmbTopology::FrameTransformEnum& transformEnum, const Pose& pose);

  const WheelVelocities& getWheelVelocities() const;
  WheelVelocities& getWheelVelocities();
  void setWheelVelocities(const WheelVelocities& wheelVelocities);

  const WheelTorques& getWheelTorques() const;
  WheelTorques& getWheelTorques();
  void setWheelTorques(const WheelTorques& wheelTorques);

  friend std::ostream& operator<<(std::ostream& out, const SmbState& state);

//TODO need these poses?
 protected:
  Pose poseMapToOdom_;
  Pose poseMapGaToOdom_;

  WheelVelocities wheelVelocities_;
  WheelTorques wheelTorques_;
};

} /* namespace smb_model */
