/*!
 * @file    containers.hpp
 * @author  Dario Bellicoso
 * @date    Apr 20, 2016
 * @version 1.0
 */

#pragma once

// romo
#include <romo/common/RobotDescription.hpp>

// std utils
#include <std_utils/std_utils.hpp>

namespace romo_std {

/* ActuatorCommand */
template <typename ConcreteDescription_, typename ActuatorCommand_>
using ActuatorCommandContainer = std_utils::EnumArray< typename romo::RobotDescription<ConcreteDescription_>::ActuatorEnum, ActuatorCommand_>;

template <typename ConcreteDescription_, typename ActuatorCommand_>
using ActuatorCommandPtrContainer = std_utils::EnumArray<typename romo::RobotDescription<ConcreteDescription_>::ActuatorEnum, ActuatorCommand_*>;

template <typename ConcreteDescription_, typename ActuatorCommand_>
using ActuatorCommandNodeContainer = std_utils::EnumMap<typename romo::RobotDescription<ConcreteDescription_>::ActuatorNodeEnum, ActuatorCommand_>;

template <typename ConcreteDescription_, typename ActuatorCommand_>
using ActuatorCommandPtrNodeContainer = std_utils::EnumMap<typename romo::RobotDescription<ConcreteDescription_>::ActuatorNodeEnum, ActuatorCommand_*>;

template <typename ConcreteDescription_, typename ActuatorCommand_>
using ActuatorCommandBranchContainer = std_utils::EnumMap<typename romo::RobotDescription<ConcreteDescription_>::LimbEnum, ActuatorCommand_>;

template <typename ConcreteDescription_, typename ActuatorCommand_>
using ActuatorCommandPtrBranchContainer = std_utils::EnumMap<typename romo::RobotDescription<ConcreteDescription_>::LimbEnum, ActuatorCommand_*>;

template <typename ConcreteDescription_, typename ActuatorCommand_>
using ActuatorCommandPtrBranchNodeContainer = std_utils::EnumMap<typename romo::RobotDescription<ConcreteDescription_>::LimbEnum, ActuatorCommandPtrNodeContainer<ConcreteDescription_, ActuatorCommand_>>;

template <typename ConcreteDescription_, typename ActuatorCommand_>
using ActuatorCommandPtrNodeBranchContainer = std_utils::EnumMap<typename romo::RobotDescription<ConcreteDescription_>::ActuatorNodeEnum , ActuatorCommandPtrBranchContainer<ConcreteDescription_, ActuatorCommand_>>;


/* ActuatorState */
template <typename ConcreteDescription_, typename ActuatorState_>
using ActuatorStateContainer = std_utils::EnumArray<typename romo::RobotDescription<ConcreteDescription_>::ActuatorEnum, ActuatorState_>;

template <typename ConcreteDescription_, typename ActuatorState_>
using ActuatorStatePtrContainer = std_utils::EnumArray<typename romo::RobotDescription<ConcreteDescription_>::ActuatorEnum, ActuatorState_*>;

template <typename ConcreteDescription_, typename ActuatorState_>
using ActuatorStateNodeContainer = std_utils::EnumMap<typename romo::RobotDescription<ConcreteDescription_>::ActuatorNodeEnum, ActuatorState_>;

template <typename ConcreteDescription_, typename ActuatorState_>
using ActuatorStatePtrNodeContainer = std_utils::EnumMap<typename romo::RobotDescription<ConcreteDescription_>::ActuatorNodeEnum, ActuatorState_*>;

template <typename ConcreteDescription_, typename ActuatorState_>
using ActuatorStateBranchContainer = std_utils::EnumMap<typename romo::RobotDescription<ConcreteDescription_>::LimbEnum, ActuatorState_>;

template <typename ConcreteDescription_, typename ActuatorState_>
using ActuatorStatePtrBranchContainer = std_utils::EnumMap<typename romo::RobotDescription<ConcreteDescription_>::LimbEnum, ActuatorState_*>;

template <typename ConcreteDescription_, typename ActuatorState_>
using ActuatorStatePtrBranchNodeContainer = std_utils::EnumMap<typename romo::RobotDescription<ConcreteDescription_>::LimbEnum, ActuatorStatePtrNodeContainer<ConcreteDescription_, ActuatorState_>>;

template <typename ConcreteDescription_, typename ActuatorState_>
using ActuatorStatePtrNodeBranchContainer = std_utils::EnumMap<typename romo::RobotDescription<ConcreteDescription_>::ActuatorNodeEnum , ActuatorStatePtrBranchContainer<ConcreteDescription_, ActuatorState_>>;


/* ActuatorReading */
template <typename ConcreteDescription_, typename ActuatorReading_>
using ActuatorReadingContainer = std_utils::EnumArray<typename romo::RobotDescription<ConcreteDescription_>::ActuatorEnum, ActuatorReading_>;

template <typename ConcreteDescription_, typename ActuatorReading_>
using ActuatorReadingPtrContainer = std_utils::EnumArray<typename romo::RobotDescription<ConcreteDescription_>::ActuatorEnum, ActuatorReading_*>;

template <typename ConcreteDescription_, typename ActuatorReading_>
using ActuatorReadingNodeContainer = std_utils::EnumMap<typename romo::RobotDescription<ConcreteDescription_>::ActuatorNodeEnum, ActuatorReading_>;

template <typename ConcreteDescription_, typename ActuatorReading_>
using ActuatorReadingPtrNodeContainer = std_utils::EnumMap<typename romo::RobotDescription<ConcreteDescription_>::ActuatorNodeEnum, ActuatorReading_*>;

template <typename ConcreteDescription_, typename ActuatorReading_>
using ActuatorReadingBranchContainer = std_utils::EnumMap<typename romo::RobotDescription<ConcreteDescription_>::LimbEnum, ActuatorReading_>;

template <typename ConcreteDescription_, typename ActuatorReading_>
using ActuatorReadingPtrBranchContainer = std_utils::EnumMap<typename romo::RobotDescription<ConcreteDescription_>::LimbEnum, ActuatorReading_*>;

template <typename ConcreteDescription_, typename ActuatorReading_>
using ActuatorReadingPtrBranchNodeContainer = std_utils::EnumMap<typename romo::RobotDescription<ConcreteDescription_>::LimbEnum, ActuatorReadingPtrNodeContainer<ConcreteDescription_, ActuatorReading_>>;

template <typename ConcreteDescription_, typename ActuatorReading_>
using ActuatorReadingPtrNodeBranchContainer = std_utils::EnumMap<typename romo::RobotDescription<ConcreteDescription_>::ActuatorNodeEnum , ActuatorReadingPtrBranchContainer<ConcreteDescription_, ActuatorReading_>>;


/* Perfect torque source container */
template <typename ConcreteDescription_, typename ActuatorPerfectTorqueSource_>
using ActuatorPerfectTorqueSourceRobotContainer = std_utils::EnumArray<typename romo::RobotDescription<ConcreteDescription_>::ActuatorEnum, ActuatorPerfectTorqueSource_>;

} // namespace romo_std

