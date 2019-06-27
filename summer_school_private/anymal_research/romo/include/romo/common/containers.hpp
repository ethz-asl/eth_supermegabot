/*!
 * @file    containers.hpp
 * @author  Dario Bellicoso
 * @date    Apr 20, 2016
 * @version 1.0
 */

#pragma once

// std utils
#include <std_utils/std_utils.hpp>

// romo
#include "romo/common/Contact.hpp"
#include "romo/common/RigidBody.hpp"
#include "romo/common/RobotDescription.hpp"

// Boost
#include <boost/range/adaptor/map.hpp>

// Stl
#include <memory>

namespace romo {

/* Bodies */
template<typename ConcreteDescription_>
using RigidBodyContainer = std_utils::EnumArray< typename RobotDescription<ConcreteDescription_>::BodyEnum, RigidBody<ConcreteDescription_>>;

template<typename ConcreteDescription_>
using RigidBodyShPtrContainer = std_utils::EnumArray< typename RobotDescription<ConcreteDescription_>::BodyEnum, std::shared_ptr<RigidBody<ConcreteDescription_>>>;

template<typename ConcreteDescription_>
using RigidBodyShPtrBranchContainer = std_utils::EnumMap< typename RobotDescription<ConcreteDescription_>::BranchEnum, std::shared_ptr<RigidBody<ConcreteDescription_>>>;

template<typename ConcreteDescription_>
using RigidBodyShPtrNodeContainer = std_utils::EnumMap< typename RobotDescription<ConcreteDescription_>::BodyNodeEnum, std::shared_ptr<RigidBody<ConcreteDescription_>>>;

template<typename ConcreteDescription_>
using RigidBodyShPtrBranchNodeContainer = std_utils::EnumMap< typename RobotDescription<ConcreteDescription_>::BranchEnum, RigidBodyShPtrNodeContainer<ConcreteDescription_>>;

template<typename ConcreteDescription_>
using RigidBodyShPtrNodeBranchContainer = std_utils::EnumMap< typename RobotDescription<ConcreteDescription_>::BodyNodeEnum, RigidBodyShPtrBranchContainer<ConcreteDescription_>>;

/* Contact */
template<typename ConcreteDescription_>
using ContactContainer = std_utils::EnumArray< typename RobotDescription<ConcreteDescription_>::ContactEnum, Contact<ConcreteDescription_>>;

template<typename ConcreteDescription_>
using ContactShPtrContainer = std_utils::EnumArray< typename RobotDescription<ConcreteDescription_>::ContactEnum, std::shared_ptr<Contact<ConcreteDescription_>>>;


} // namespace romo
