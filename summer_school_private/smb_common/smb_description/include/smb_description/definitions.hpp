/*!
 * @file     definitions.hpp
 * @author   Koen Kraemer
 * @date     Aug 1, 2018
 */

#pragma once

namespace smb_description {

struct SmbDefinitions {
  // template <SmbTopology::LimbEnum limb, SmbTopology::BodyEnum body>
  // using limbToBodyKV = std_utils::KeyValuePair<SmbTopology::LimbEnum, SmbTopology::BodyEnum, limb, body>;
  // using mapLimbToHip =
  // std_utils::CompileTimeMap< SmbTopology::LimbEnum, SmbTopology::BodyEnum,
  //                            limbToBodyKV<SmbTopology::LimbEnum::LF_LEG, SmbTopology::BodyEnum::LF_HIP>,
  //                            limbToBodyKV<SmbTopology::LimbEnum::RF_LEG, SmbTopology::BodyEnum::RF_HIP>,
  //                            limbToBodyKV<SmbTopology::LimbEnum::LH_LEG, SmbTopology::BodyEnum::LH_HIP>,
  //                            limbToBodyKV<SmbTopology::LimbEnum::RH_LEG, SmbTopology::BodyEnum::RH_HIP> >;
  // using mapLimbToThigh =
  // std_utils::CompileTimeMap< SmbTopology::LimbEnum, SmbTopology::BodyEnum,
  //                            limbToBodyKV<SmbTopology::LimbEnum::LF_LEG, SmbTopology::BodyEnum::LF_THIGH>,
  //                            limbToBodyKV<SmbTopology::LimbEnum::RF_LEG, SmbTopology::BodyEnum::RF_THIGH>,
  //                            limbToBodyKV<SmbTopology::LimbEnum::LH_LEG, SmbTopology::BodyEnum::LH_THIGH>,
  //                            limbToBodyKV<SmbTopology::LimbEnum::RH_LEG, SmbTopology::BodyEnum::RH_THIGH> >;
  // using mapLimbToShank =
  // std_utils::CompileTimeMap< SmbTopology::LimbEnum, SmbTopology::BodyEnum,
  //                            limbToBodyKV<SmbTopology::LimbEnum::LF_LEG, SmbTopology::BodyEnum::LF_SHANK>,
  //                            limbToBodyKV<SmbTopology::LimbEnum::RF_LEG, SmbTopology::BodyEnum::RF_SHANK>,
  //                            limbToBodyKV<SmbTopology::LimbEnum::LH_LEG, SmbTopology::BodyEnum::LH_SHANK>,
  //                            limbToBodyKV<SmbTopology::LimbEnum::RH_LEG, SmbTopology::BodyEnum::RH_SHANK> >;
  // using mapLimbToFoot =
  // std_utils::CompileTimeMap< SmbTopology::LimbEnum, SmbTopology::BodyEnum,
  //                            limbToBodyKV<SmbTopology::LimbEnum::LF_LEG, SmbTopology::BodyEnum::LF_FOOT>,
  //                            limbToBodyKV<SmbTopology::LimbEnum::RF_LEG, SmbTopology::BodyEnum::RF_FOOT>,
  //                            limbToBodyKV<SmbTopology::LimbEnum::LH_LEG, SmbTopology::BodyEnum::LH_FOOT>,
  //                            limbToBodyKV<SmbTopology::LimbEnum::RH_LEG, SmbTopology::BodyEnum::RH_FOOT> >;

  // template <SmbTopology::LimbEnum limb, SmbTopology::LateralEnum lateral>
  // using limbToLateralKV = std_utils::KeyValuePair<SmbTopology::LimbEnum, SmbTopology::LateralEnum, limb, lateral>;
  // using mapLimbToLateral =
  // std_utils::CompileTimeMap< SmbTopology::LimbEnum, SmbTopology::LateralEnum,
  //                            limbToLateralKV<SmbTopology::LimbEnum::LF_LEG, SmbTopology::LateralEnum::LEFT>,
  //                            limbToLateralKV<SmbTopology::LimbEnum::RF_LEG, SmbTopology::LateralEnum::RIGHT>,
  //                            limbToLateralKV<SmbTopology::LimbEnum::LH_LEG, SmbTopology::LateralEnum::LEFT>,
  //                            limbToLateralKV<SmbTopology::LimbEnum::RH_LEG, SmbTopology::LateralEnum::RIGHT> >;

  // template <SmbTopology::LimbEnum limb, SmbTopology::LongitudinalEnum longitudinal>
  // using limbToLongitudinalKV = std_utils::KeyValuePair<SmbTopology::LimbEnum, SmbTopology::LongitudinalEnum, limb, longitudinal>;
  // using mapLimbToLongitudinal =
  // std_utils::CompileTimeMap< SmbTopology::LimbEnum, SmbTopology::LongitudinalEnum,
  //                            limbToLongitudinalKV<SmbTopology::LimbEnum::LF_LEG, SmbTopology::LongitudinalEnum::FORE>,
  //                            limbToLongitudinalKV<SmbTopology::LimbEnum::RF_LEG, SmbTopology::LongitudinalEnum::FORE>,
  //                            limbToLongitudinalKV<SmbTopology::LimbEnum::LH_LEG, SmbTopology::LongitudinalEnum::HIND>,
  //                            limbToLongitudinalKV<SmbTopology::LimbEnum::RH_LEG, SmbTopology::LongitudinalEnum::HIND> >;

  // Degrees of freedom
  inline static constexpr unsigned int getNumLegsImpl() { return 0; }
  inline static constexpr unsigned int getNumArmsImpl() { return 0; }
  inline static constexpr unsigned int getNumDofImpl() { return 6; }

  inline static constexpr unsigned int getNumDofLimbImpl(SmbTopology::LimbEnum limb) {
      return SmbTopology::mapLimbToNDof::at(limb);
  }

  inline static constexpr unsigned int getNumDofLimbImpl() { return 0; }

  inline static constexpr unsigned int getBranchStartIndexInUImpl(SmbTopology::BranchEnum branch) {
    return SmbTopology::mapBranchToStartIndexInU::at(branch);
  }

  inline static constexpr unsigned int getLimbStartIndexInJImpl(SmbTopology::LimbEnum limb) {
     return SmbTopology::mapLimbToStartIndexInJ::at(limb);
  }

  inline static constexpr SmbTopology::BodyEnum getBranchStartBodyImpl(SmbTopology::BranchEnum branch) {
    return SmbTopology::mapBranchToStartBody::at(branch);
  }

  inline static constexpr SmbTopology::BodyEnum getBranchEndBodyImpl(SmbTopology::BranchEnum branch) {
    return SmbTopology::mapBranchToEndBody::at(branch);
  }

  inline static constexpr unsigned int getNumWheels() { return static_cast<unsigned int>(SmbTopology::SmbActuatorEnum::SIZE); }

  // Base degrees of freedom
  inline static constexpr unsigned int getBaseGeneralizedCoordinatesDimensionImpl() { return 7; }
  inline static constexpr unsigned int getBaseGeneralizedVelocitiesDimensionImpl() { return 6; }
};

} /* smb_description */