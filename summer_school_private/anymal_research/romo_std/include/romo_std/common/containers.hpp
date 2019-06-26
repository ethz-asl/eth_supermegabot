/*!
 * @file    containers.hpp
 * @author  Christian Gehring, Dario Bellicoso
 * @date    March 23, 2015
 * @version 1.0
 */

#pragma once

// std utils
#include <std_utils/std_utils.hpp>

// romo
#include <romo_std/ContactImpl.hpp>

// STL
#include <memory>

namespace romo_std {

/* Contacts */
template <typename ConcreteDescription_, typename RobotState_>
using ContactContainer = std_utils::EnumArray<typename romo::RobotDescription<ConcreteDescription_>::ContactEnum , romo_std::ContactImpl<ConcreteDescription_, RobotState_>>;

template <typename ConcreteDescription_, typename RobotState_>
using ContactShPtrContainer = std_utils::EnumArray<typename romo::RobotDescription<ConcreteDescription_>::ContactEnum , std::shared_ptr<romo_std::ContactImpl<ConcreteDescription_, RobotState_>>>;

} // namespace romo_std
