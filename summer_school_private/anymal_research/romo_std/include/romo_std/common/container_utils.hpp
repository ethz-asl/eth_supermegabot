/*
 * container_utils.hpp
 *
 *  Created on: Apr 29, 2016
 *      Author: Dario Bellicoso
 */

#pragma once

// romo std
#include <romo_std/common/containers.hpp>

// std utils
#include <std_utils/std_utils.hpp>

namespace romo_std {

template<typename ConcreteDescription_, typename KeyEnum_, typename Item_>
static void fillContainer(std_utils::EnumArray<KeyEnum_, Item_>& container) {
  for (const auto & key : romo::RobotDescription<ConcreteDescription_>::template getKeys<KeyEnum_>()) {
    container[key.getEnum()] = Item_();
  }
}

template<typename ConcreteDescription_, typename KeyEnum_, typename Item_>
static void fillPtrContainer(std_utils::EnumArray<KeyEnum_, Item_*>& containerPtr,
                             std_utils::EnumArray<KeyEnum_, Item_>& container) {
  for (const auto & key : romo::RobotDescription<ConcreteDescription_>::template getKeys<KeyEnum_>()) {
    containerPtr[key.getEnum()] = &container[key.getEnum()];
  }
}

template<typename ConcreteDescription_, typename Item_, typename ContainerEnum_, typename NodeEnum_, typename BranchEnum_>
static void fillPtrBranchContainer(std_utils::EnumArray<NodeEnum_, Item_*>& branchContainer,
                                   BranchEnum_ branchId,
                                   std_utils::EnumArray<Item_*, ContainerEnum_>& container)
{
  using RD = romo::RobotDescription<ConcreteDescription_>;

  for (const auto & key : RD::template getKeys<ContainerEnum_>()) {
    if (RD::template mapEnums<BranchEnum_>(key.getEnum()) == branchId) {
      branchContainer[RD::template mapEnums<NodeEnum_>(key.getEnum())] = &container.at(key.getEnum());
    }
  }
}

template<typename ConcreteDescription_, typename Item_, typename ContainerEnum_, typename NodeEnum_, typename BranchEnum_>
static void fillPtrNodeContainer(std_utils::EnumArray<BranchEnum_, Item_*>&  nodeContainer,
                                 NodeEnum_ nodeId,
                                 std_utils::EnumArray<ContainerEnum_, Item_>& container)
{
  using RD = romo::RobotDescription<ConcreteDescription_>;

  for (const auto & key : RD::template getKeys<ContainerEnum_>()) {
    if (RD::template mapEnums<NodeEnum_>(key.getEnum()) == nodeId) {
      nodeContainer[RD::template mapEnums<BranchEnum_>(key.getEnum())] = &container.at(key.getEnum());
    }
  }
}

template<typename ConcreteDescription_, typename Item_, typename ContainerEnum_, typename NodeEnum_, typename BranchEnum_>
static void fillBranchNodePtrContainer(std_utils::EnumMap<BranchEnum_, std_utils::EnumMap<NodeEnum_, Item_*>>& branchNodeContainer,
                                       std_utils::EnumArray<ContainerEnum_, Item_>& container) {

  using RD = romo::RobotDescription<ConcreteDescription_>;

  for (const auto & key : RD::template getKeys<ContainerEnum_>()) {
    branchNodeContainer[RD::template mapEnums<BranchEnum_>(key.getEnum())][RD::template mapEnums<NodeEnum_>(key.getEnum())] = &container.at(key.getEnum());
  }
}

template<typename ConcreteDescription_, typename Item_, typename ContainerEnum_, typename NodeEnum_, typename BranchEnum_>
static void fillBranchNodePtrContainer(std_utils::EnumMap<BranchEnum_, std_utils::EnumMap<NodeEnum_, Item_*>>& branchNodeContainer,
                                       std_utils::EnumArray<ContainerEnum_, Item_*>& container) {

  using RD = romo::RobotDescription<ConcreteDescription_>;

  for (const auto & key : RD::template getKeys<ContainerEnum_>()) {
    branchNodeContainer[RD::template mapEnums<BranchEnum_>(key.getEnum())][RD::template mapEnums<NodeEnum_>(key.getEnum())] = container.at(key.getEnum());
  }
}

template<typename ConcreteDescription_, typename Item_, typename ContainerEnum_, typename NodeEnum_, typename BranchEnum_>
static void fillBranchNodeShPtrContainer(std_utils::EnumMap<BranchEnum_, std_utils::EnumMap<NodeEnum_, std::shared_ptr<Item_>>>& branchNodeContainer,
                                         std_utils::EnumArray<ContainerEnum_, std::shared_ptr<Item_>>& container) {

  using RD = romo::RobotDescription<ConcreteDescription_>;

  for (const auto & key : RD::template getKeys<ContainerEnum_>()) {
    branchNodeContainer[RD::template mapEnums<BranchEnum_>(key.getEnum())][RD::template mapEnums<NodeEnum_>(key.getEnum())] = container.at(key.getEnum());
  }
}

template<typename ConcreteDescription_, typename Item_, typename ContainerEnum_, typename NodeEnum_, typename BranchEnum_>
static void fillNodeBranchPtrContainer(std_utils::EnumMap<NodeEnum_, std_utils::EnumMap<BranchEnum_, Item_*>>& branchNodeContainer,
                                       std_utils::EnumArray<ContainerEnum_, Item_>& container) {

  using RD = romo::RobotDescription<ConcreteDescription_>;

  for (const auto & key : RD::template getKeys<ContainerEnum_>()) {
    branchNodeContainer[RD::template mapEnums<NodeEnum_>(key.getEnum())][RD::template mapEnums<BranchEnum_>(key.getEnum())] = &container.at(key.getEnum());
  }
}

template<typename ConcreteDescription_, typename Item_, typename ContainerEnum_, typename NodeEnum_, typename BranchEnum_>
static void fillNodeBranchPtrContainer(std_utils::EnumMap<NodeEnum_, std_utils::EnumMap<BranchEnum_, Item_*>>& branchNodeContainer,
                                       std_utils::EnumArray<ContainerEnum_, Item_*>& container) {

  using RD = romo::RobotDescription<ConcreteDescription_>;

  for (const auto & key : RD::template getKeys<ContainerEnum_>()) {
    branchNodeContainer[RD::template mapEnums<NodeEnum_>(key.getEnum())][RD::template mapEnums<BranchEnum_>(key.getEnum())] = container.at(key.getEnum());
  }
}

template<typename ConcreteDescription_, typename Item_, typename ContainerEnum_, typename NodeEnum_, typename BranchEnum_>
static void fillNodeBranchShPtrContainer(std_utils::EnumMap<NodeEnum_, std_utils::EnumMap<BranchEnum_, std::shared_ptr<Item_>>>& branchNodeContainer,
                                         std_utils::EnumArray<ContainerEnum_, std::shared_ptr<Item_>>& container) {

  using RD = romo::RobotDescription<ConcreteDescription_>;

  for (const auto & key : RD::template getKeys<ContainerEnum_>()) {
    branchNodeContainer[RD::template mapEnums<NodeEnum_>(key.getEnum())][RD::template mapEnums<BranchEnum_>(key.getEnum())] = container.at(key.getEnum());
  }
}




} /* namespace romo_std */
