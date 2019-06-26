/**
 * @file    containers.hpp
 * @author  Koen Kraemer
 * @date    Aug 2, 2018
 */

#pragma once

// smb model
//#include <smb_model/common/typedefs.hpp>
// actuator
// #include <series_elastic_actuator/SeActuatorCommand.hpp>
// #include <series_elastic_actuator/SeActuatorReading.hpp>

// smb wheels
#include <smb_common/SmbWheelCommand.hpp>
#include <smb_common/SmbWheelReading.hpp>
//
//// romo std
//#include "romo_std/common/actuator_containers.hpp"

using ConcreteDescription = romo::ConcreteDescription<smb_description::SmbDefinitions, smb_description::SmbTopology>;

namespace smb_model {

// using ActuatorCommandRobotContainer =
//     romo_std::ActuatorCommandContainer<ConcreteDescription, series_elastic_actuator::SeActuatorCommand>;
// using ActuatorCommandPtrBranchNodeContainer =
//     romo_std::ActuatorCommandPtrBranchNodeContainer<ConcreteDescription, series_elastic_actuator::SeActuatorCommand>;
// using ActuatorCommandPtrNodeBranchContainer =
//     romo_std::ActuatorCommandPtrNodeBranchContainer<ConcreteDescription, series_elastic_actuator::SeActuatorCommand>;

// using ActuatorReadingRobotContainer =
//     romo_std::ActuatorReadingContainer<ConcreteDescription, series_elastic_actuator::SeActuatorReading>;
// using ActuatorReadingPtrBranchNodeContainer =
//     romo_std::ActuatorReadingPtrBranchNodeContainer<ConcreteDescription, series_elastic_actuator::SeActuatorReading>;
// using ActuatorReadingPtrNodeBranchContainer =
//     romo_std::ActuatorReadingPtrNodeBranchContainer<ConcreteDescription, series_elastic_actuator::SeActuatorReading>;


  using SmbWheelReadings = std_utils::EnumArray<ConcreteDescription::ConcreteTopology::SmbActuatorEnum, smb_common::SmbWheelReading>;
  using SmbWheelCommands = std_utils::EnumArray<ConcreteDescription::ConcreteTopology::SmbActuatorEnum, smb_common::SmbWheelCommand>;

} /* namespace smb_model */
