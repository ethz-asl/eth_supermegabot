/**
 * @author Gabriel Hottiger
 * @brief A container for the joint limits
 */

#pragma once

#include "romo/common/RobotDescription.hpp"
#include "romo/common/phys_typedefs.hpp"
#include "romo/common/robot_state_typedefs.hpp"

namespace romo {

template <typename ConcreteDescription_, typename RobotState_>
class LimitsInterface {
 public:
  using RD = romo::RobotDescription<ConcreteDescription_>;
  using JointEnum = typename RD::JointEnum;
  using ActuatorEnum = typename RD::ActuatorEnum;
  using JointPositions = typename romo::internal::JointPositions<RD::getJointsDimension()>;
  using JointVelocities = typename romo::internal::JointVelocities<RD::getJointsDimension()>;

  LimitsInterface() = default;
  virtual ~LimitsInterface() = default;

  virtual void init() = 0;
  virtual bool update(const JointPositions& jointPositions, const JointVelocities& jointVelocities) = 0;
  virtual bool addVariablesToLog(const std::string& ns = "limits/") const { return true; }
  virtual void printLimits() const = 0;

  virtual double getActuatorMinPosition(ActuatorEnum actuator) const = 0;
  virtual double getActuatorMaxPosition(ActuatorEnum actuator) const = 0;
  virtual double getActuatorMinVelocity(ActuatorEnum actuator) const = 0;
  virtual double getActuatorMaxVelocity(ActuatorEnum actuator) const = 0;
  virtual double getActuatorMinEffort(ActuatorEnum actuator) const = 0;
  virtual double getActuatorMaxEffort(ActuatorEnum actuator) const = 0;

  virtual double getJointMinPosition(JointEnum joint) const = 0;
  virtual double getJointMaxPosition(JointEnum joint) const = 0;
  virtual double getJointMinVelocity(JointEnum joint) const = 0;
  virtual double getJointMaxVelocity(JointEnum joint) const = 0;
  virtual double getJointMinEffort(JointEnum joint) const = 0;
  virtual double getJointMaxEffort(JointEnum joint) const = 0;
};

}  // namespace romo