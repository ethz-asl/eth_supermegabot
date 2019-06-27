/*
 * SmbWheelCommandTranslator.hpp
 *
 *  Created on: Oct 6, 2018
 *      Author: Koen Kraemer
 */

#pragma once

// loco
#include <loco/common/typedefs.hpp>

// message logger
#include <message_logger/message_logger.hpp>

namespace smb_description {

static double computeDesiredWheelVelocity(SmbTopology::ActuatorEnum smbActuatorEnum, double baseLinearVelocityXInBaseFrame, double baseAngularVelocityZInBaseFrame, double baseWidth, double wheelRadius) {
  double wheelVelocity;
  // TODO add integrator
  switch (smbActuatorEnum) {
    case SmbTopology::ActuatorEnum::LF_WHEEL :
    case SmbTopology::ActuatorEnum::LH_WHEEL :
      wheelVelocity = (baseLinearVelocityXInBaseFrame - baseAngularVelocityZInBaseFrame * baseWidth / 4.0) / wheelRadius;
      break;
    case SmbTopology::ActuatorEnum::RF_WHEEL :
    case SmbTopology::ActuatorEnum::RH_WHEEL :
      wheelVelocity = (baseLinearVelocityXInBaseFrame + baseAngularVelocityZInBaseFrame * baseWidth / 4.0) / wheelRadius;
      break;
    default:
    MELO_WARN_THROTTLE_STREAM(1.0, "[SmbWheelCommandTranslator] Unknown ActuatorEnum");
      wheelVelocity = 0.0;
      break;
  }

  return wheelVelocity;
}

double computeDesiredWheelTorque(SmbTopology::ActuatorEnum smbActuatorEnum, double baseForceXInBaseFrame, double baseTorqueZInBaseFrame, double baseWidth, double wheelRadius) {
  double wheelTorque;
  switch (smbActuatorEnum) {
    case SmbTopology::ActuatorEnum::LF_WHEEL :
    case SmbTopology::ActuatorEnum::LH_WHEEL :
      wheelTorque = (0.5 * baseForceXInBaseFrame - baseTorqueZInBaseFrame / baseWidth) * wheelRadius;
      break;
    case SmbTopology::ActuatorEnum::RF_WHEEL :
    case SmbTopology::ActuatorEnum::RH_WHEEL :
      wheelTorque = (0.5 * baseForceXInBaseFrame + baseTorqueZInBaseFrame / baseWidth) * wheelRadius;
      break;
    default:
    MELO_WARN_THROTTLE_STREAM(1.0, "[SmbWheelCommandTranslator] Unknown ActuatorEnum");
      wheelTorque = 0.0;
      break;
  }

  return wheelTorque;
}

} /* namespace smb_description */
