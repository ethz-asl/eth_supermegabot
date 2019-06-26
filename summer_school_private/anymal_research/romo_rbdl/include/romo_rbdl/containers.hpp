/*!
 * @file    containers.hpp
 * @author  Dario Bellicoso, Christian Gehring
 * @date    March 23, 2015
 * @version 1.0
 */

#pragma once

// std utils
#include <std_utils/std_utils.hpp>

// romo
#include <romo_rbdl/RigidBodyRbdl.hpp>
#include <romo/common/RobotDescription.hpp>

namespace romo_rbdl {

// Access the container by BodyeEnum as enum key, e.g. container[BodyEnum::SomeBody]. Returns a copy of the item.
template<typename ConcreteDescription_>
using RigidBodyRbdlContainer = std_utils::EnumArray< typename romo::RobotDescription<ConcreteDescription_>::BodyEnum, RigidBodyRbdl<ConcreteDescription_>>;

// Access the container by BodyEnum as enum key, e.g. container[BodyEnum::SomeBody]. Returns a pointer to the item.
template<typename ConcreteDescription_>
using RigidBodyRbdlShPtrContainer = std_utils::EnumArray< typename romo::RobotDescription<ConcreteDescription_>::BodyEnum, std::shared_ptr<RigidBodyRbdl<ConcreteDescription_>>>;

// Access the container by BodyNodeEnum as enum key, e.g. container[BodyNodeEnum::SomeNode]. Returns a pointer to the item.
template<typename ConcreteDescription_>
using RigidBodyRbdlShPtrNodeContainer = std_utils::EnumMap< typename romo::RobotDescription<ConcreteDescription_>::BodyNodeEnum, std::shared_ptr<RigidBodyRbdl<ConcreteDescription_>>>;

// Access the container by BranchEnum as enum key, e.g. container[BranchEnum::SomeBranch]. Returns a pointer to the item.
template<typename ConcreteDescription_>
using RigidBodyRbdlShPtrBranchContainer = std_utils::EnumMap< typename romo::RobotDescription<ConcreteDescription_>::BranchEnum, std::shared_ptr<RigidBodyRbdl<ConcreteDescription_>>>;

// Access the container by BranchEnum and BodyNode as enum keys, e.g. container[BranchEnum::SomeBranch][BodyNodeEnum::SomeNode]. Returns a pointer to the item.
template<typename ConcreteDescription_>
using RigidBodyRbdlShPtrBranchNodeContainer = std_utils::EnumMap< typename romo::RobotDescription<ConcreteDescription_>::BranchEnum, RigidBodyRbdlShPtrNodeContainer<ConcreteDescription_>>;

// Access the container by BodyNode and BranchEnum as enum keys, e.g. container[BodyNodeEnum::SomeNode][BranchEnum::SomeBranch]. Returns a pointer to the item.
template<typename ConcreteDescription_>
using RigidBodyRbdlShPtrNodeBranchContainer = std_utils::EnumMap< typename romo::RobotDescription<ConcreteDescription_>::BodyNodeEnum, RigidBodyRbdlShPtrBranchContainer<ConcreteDescription_>>;

} // namespace romo_rbdl