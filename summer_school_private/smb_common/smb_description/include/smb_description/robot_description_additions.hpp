/*!
 * @file     robot_description_additions.hpp
 * @author   Koen Kraemer
 * @date     Aug 1, 2018
 */

#pragma once

#include <smb_description/topology.hpp>

namespace romo {
namespace internal {

//template <>
//struct KeysHelper<smb_description::SmbTopology, typename smb_description::SmbTopology::FrameTransformEnum> {
//  inline static constexpr std_utils::KeyArray<typename smb_description::SmbTopology::FrameTransformEnum> keys() {
//    return smb_description::SmbTopology::frameTransformKeys;
//  }
//};

// template <>
// struct KeysHelper<smb_description::SmbTopology, typename smb_description::SmbTopology::FootEnum> {
//   inline static constexpr std_utils::KeyArray<typename smb_description::SmbTopology::FootEnum> keys() {
//     return smb_description::SmbTopology::footKeys;
//   }
// };

// template <>
// struct MapEnumHelper<typename smb_description::SmbTopology::FootEnum, 
//                      typename smb_description::SmbTopology::BranchEnum, 
//                      smb_description::SmbTopology> 
// {
//   static constexpr typename smb_description::SmbTopology::BranchEnum map( typename smb_description::SmbTopology::FootEnum footEnum) {
//     return smb_description::SmbTopology::mapFootEnumToBranchEnum::at(footEnum);
//   }
// };

// template <>
// struct MapEnumHelper<typename smb_description::SmbTopology::FootEnum, 
//                      typename smb_description::SmbTopology::ContactEnum, 
//                      smb_description::SmbTopology> 
// {
//   static constexpr typename smb_description::SmbTopology::ContactEnum map( typename smb_description::SmbTopology::FootEnum footEnum) {
//     return smb_description::SmbTopology::mapFootEnumToContactEnum::at(footEnum);
//   }
// };

template <>
struct KeysHelper<smb_description::SmbTopology, typename smb_description::SmbTopology::SmbActuatorEnum > {
  inline static constexpr std_utils::KeyArray<typename smb_description::SmbTopology::SmbActuatorEnum> keys() {
    return smb_description::SmbTopology::smbActuatorKeys;
  }
};


} /* namespace internal */
} /* namespace romo */