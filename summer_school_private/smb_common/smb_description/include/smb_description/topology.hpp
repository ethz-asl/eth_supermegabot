/*!
 * @file     topology.hpp
 * @author   Koen Kraemer
 * @date     Aug 1, 2018
 */

#pragma once

// std_utils
#include <std_utils/std_utils.hpp>

// romo
#include <romo/common/RobotDescription.hpp>

namespace smb_description {

struct SmbTopology {

  // ----- Enums ----- //


   #define SMB_LIMB_NAMES DUMMY
//  CONSECUTIVE_ENUM_EMPTY(LimbEnum);

//  CONSECUTIVE_ENUM_FROM_LIST(LimbEnum, SMB_LIMB_NAMES);
//  using ct_limbKeys =
//    std_utils::CompileTimeKeys<LimbEnum, std_utils::CompileTimeKey<LimbEnum, LimbEnum::LF_LEG, ct_string("LF_LEG")>,
//                                   std_utils::CompileTimeKey<LimbEnum, LimbEnum::RF_LEG, ct_string("RF_LEG")>,
//                                   std_utils::CompileTimeKey<LimbEnum, LimbEnum::LH_LEG, ct_string("LH_LEG")>,
//                                   std_utils::CompileTimeKey<LimbEnum, LimbEnum::RH_LEG, ct_string("RH_LEG")>>;
//  static constexpr auto limbKeys = ct_limbKeys::getKeysArray();

 CONSECUTIVE_ENUM_FROM_LIST(LimbEnum, SMB_LIMB_NAMES);
 using ct_limbKeys =
   std_utils::CompileTimeKeys<LimbEnum, std_utils::CompileTimeKey<LimbEnum, LimbEnum::DUMMY, ct_string("DUMMY")>>;
 static constexpr auto limbKeys = ct_limbKeys::getKeysArray();

  #define SMB_BRANCH_NAMES BASE
  CONSECUTIVE_ENUM_FROM_LIST(BranchEnum, SMB_BRANCH_NAMES);
  using ct_branchKeys =
      std_utils::CompileTimeKeys<BranchEnum, std_utils::CompileTimeKey<BranchEnum, BranchEnum::BASE, ct_string("BASE")>>;
  static constexpr auto branchKeys = ct_branchKeys::getKeysArray();

//  #define SMB_JOINT_NODE_NAMES DUMMY_JN
//  CONSECUTIVE_ENUM_FROM_LIST(JointNodeEnum, SMB_JOINT_NODE_NAMES);
//  using ct_jointNodeKeys =
//     std_utils::CompileTimeKeys<JointNodeEnum, std_utils::CompileTimeKey<JointNodeEnum, JointNodeEnum::DUMMY_JN, ct_string("DUMMY_JN")>>;
//  static constexpr auto jointNodeKeys = ct_jointNodeKeys::getKeysArray();

    CONSECUTIVE_ENUM_EMPTY(JointNodeEnum);
    using ct_jointNodeKeys = std_utils::CompileTimeKeys<JointNodeEnum>;
    static constexpr auto jointNodeKeys = ct_jointNodeKeys::getKeysArray();

//  #define SMB_JOINT_NAMES DUMMY_J
//  CONSECUTIVE_ENUM_FROM_LIST(JointEnum, SMB_JOINT_NAMES);
//   using ct_jointKeys =
//       std_utils::CompileTimeKeys<JointEnum, std_utils::CompileTimeKey<JointEnum, JointEnum::DUMMY_J, ct_string("DUMMY_J")>>;
//   static constexpr auto jointKeys = ct_jointKeys::getKeysArray();

  CONSECUTIVE_ENUM_EMPTY(JointEnum);
    using ct_jointKeys = std_utils::CompileTimeKeys<JointEnum>;
    static constexpr auto jointKeys = ct_jointKeys::getKeysArray();

  #define SMB_ACTUATOR_NAMES LF_WHEEL, RF_WHEEL, LH_WHEEL, RH_WHEEL
  CONSECUTIVE_ENUM_FROM_LIST(ActuatorEnum, SMB_ACTUATOR_NAMES);
  using ct_actuatorKeys =
      std_utils::CompileTimeKeys<ActuatorEnum, std_utils::CompileTimeKey<ActuatorEnum, ActuatorEnum::LF_WHEEL, ct_string("LF_WHEEL")>,
                                  std_utils::CompileTimeKey<ActuatorEnum, ActuatorEnum::RF_WHEEL, ct_string("RF_WHEEL")>,
                                  std_utils::CompileTimeKey<ActuatorEnum, ActuatorEnum::LH_WHEEL, ct_string("LH_WHEEL")>,
                                  std_utils::CompileTimeKey<ActuatorEnum, ActuatorEnum::RH_WHEEL, ct_string("RH_WHEEL")>>;
  static constexpr auto actuatorKeys = ct_actuatorKeys::getKeysArray();

  #define SMB_SPECIFIC_ACTUATOR_NAMES SMB_ACTUATOR_NAMES
  CONSECUTIVE_ENUM_FROM_LIST(SmbActuatorEnum, SMB_SPECIFIC_ACTUATOR_NAMES);
  using ct_smbActuatorKeys =
      std_utils::CompileTimeKeys<SmbActuatorEnum, std_utils::CompileTimeKey<SmbActuatorEnum, SmbActuatorEnum::LF_WHEEL, ct_string("LF_WHEEL")>,
                                  std_utils::CompileTimeKey<SmbActuatorEnum, SmbActuatorEnum::RF_WHEEL, ct_string("RF_WHEEL")>,
                                  std_utils::CompileTimeKey<SmbActuatorEnum, SmbActuatorEnum::LH_WHEEL, ct_string("LH_WHEEL")>,
                                  std_utils::CompileTimeKey<SmbActuatorEnum, SmbActuatorEnum::RH_WHEEL, ct_string("RH_WHEEL")>>;
  static constexpr auto smbActuatorKeys = ct_smbActuatorKeys::getKeysArray();

  #define SMB_ACTUATOR_NODE_NAMES WHEEL
  CONSECUTIVE_ENUM_FROM_LIST(ActuatorNodeEnum, SMB_ACTUATOR_NODE_NAMES);
  using ct_actuatorNodeKeys =
      std_utils::CompileTimeKeys<ActuatorNodeEnum, std_utils::CompileTimeKey<ActuatorNodeEnum, ActuatorNodeEnum::WHEEL, ct_string("WHEEL")>>;
  static constexpr auto actuatorNodeKeys = ct_actuatorNodeKeys::getKeysArray();    

  #define SMB_BODY_NAMES BASE
  CONSECUTIVE_ENUM_FROM_LIST(BodyEnum, SMB_BODY_NAMES);
  using ct_bodyKeys =
      std_utils::CompileTimeKeys<BodyEnum,
                                  std_utils::CompileTimeKey<BodyEnum, BodyEnum::BASE,     ct_string("base")>>;
  static constexpr auto bodyKeys = ct_bodyKeys::getKeysArray();

    #define SMB_BODY_NODE_NAMES BASE
  CONSECUTIVE_ENUM_FROM_LIST(BodyNodeEnum, SMB_BODY_NODE_NAMES);
  using ct_bodyNodeKeys =
      std_utils::CompileTimeKeys<BodyNodeEnum, std_utils::CompileTimeKey<BodyNodeEnum, BodyNodeEnum::BASE, ct_string("BASE")>>;
  static constexpr auto bodyNodeKeys = ct_bodyNodeKeys::getKeysArray();

  #define SMB_GENERALIZED_COORDINATE_NAMES X, Y, Z, Q_W, Q_X, Q_Y, Q_Z
  CONSECUTIVE_ENUM_FROM_LIST(GeneralizedCoordinatesEnum, SMB_GENERALIZED_COORDINATE_NAMES);
  using ct_generalizedCoordinateKeys =
      std_utils::CompileTimeKeys<GeneralizedCoordinatesEnum,
                                  std_utils::CompileTimeKey<GeneralizedCoordinatesEnum, GeneralizedCoordinatesEnum::X,      ct_string("X")>,
                                  std_utils::CompileTimeKey<GeneralizedCoordinatesEnum, GeneralizedCoordinatesEnum::Y,      ct_string("Y")>,
                                  std_utils::CompileTimeKey<GeneralizedCoordinatesEnum, GeneralizedCoordinatesEnum::Z,      ct_string("Z")>,
                                  std_utils::CompileTimeKey<GeneralizedCoordinatesEnum, GeneralizedCoordinatesEnum::Q_W,    ct_string("Q_W")>,
                                  std_utils::CompileTimeKey<GeneralizedCoordinatesEnum, GeneralizedCoordinatesEnum::Q_X,    ct_string("Q_X")>,
                                  std_utils::CompileTimeKey<GeneralizedCoordinatesEnum, GeneralizedCoordinatesEnum::Q_Y,    ct_string("Q_Y")>,
                                  std_utils::CompileTimeKey<GeneralizedCoordinatesEnum, GeneralizedCoordinatesEnum::Q_Z,    ct_string("Q_Z")>>;
  static constexpr auto generalizedCoordinateKeys = ct_generalizedCoordinateKeys::getKeysArray();  

  #define SMB_GENERALIZED_VELOCITY_NAMES L_X, L_Y, L_Z, A_X, A_Y, A_Z
  CONSECUTIVE_ENUM_FROM_LIST(GeneralizedVelocitiesEnum, SMB_GENERALIZED_VELOCITY_NAMES);
  using ct_generalizedVelocityKeys =
      std_utils::CompileTimeKeys<GeneralizedVelocitiesEnum,
                                  std_utils::CompileTimeKey<GeneralizedVelocitiesEnum, GeneralizedVelocitiesEnum::L_X,      ct_string("L_X")>,
                                  std_utils::CompileTimeKey<GeneralizedVelocitiesEnum, GeneralizedVelocitiesEnum::L_Y,      ct_string("L_Y")>,
                                  std_utils::CompileTimeKey<GeneralizedVelocitiesEnum, GeneralizedVelocitiesEnum::L_Z,      ct_string("L_Z")>,
                                  std_utils::CompileTimeKey<GeneralizedVelocitiesEnum, GeneralizedVelocitiesEnum::A_X,      ct_string("A_X")>,
                                  std_utils::CompileTimeKey<GeneralizedVelocitiesEnum, GeneralizedVelocitiesEnum::A_Y,      ct_string("A_Y")>,
                                  std_utils::CompileTimeKey<GeneralizedVelocitiesEnum, GeneralizedVelocitiesEnum::A_Z,      ct_string("A_Z")>>;
  static constexpr auto generalizedVelocityKeys = ct_generalizedVelocityKeys::getKeysArray();

  #define SMB_CONTACT_NAMES BASE
  CONSECUTIVE_ENUM_FROM_LIST(ContactEnum, SMB_CONTACT_NAMES);
  using ct_contactKeys =
      std_utils::CompileTimeKeys<ContactEnum, 
                                  std_utils::CompileTimeKey<ContactEnum, ContactEnum::BASE, ct_string("base")>>;
  static constexpr auto contactKeys = ct_contactKeys::getKeysArray();

  #define SMB_CONTACT_STATE_NAMES OPEN, CLOSED, SLIPPING
  CONSECUTIVE_ENUM_FROM_LIST(ContactStateEnum, SMB_CONTACT_STATE_NAMES);
  using ct_contactStateKeys =
      std_utils::CompileTimeKeys<ContactStateEnum, 
                                  std_utils::CompileTimeKey<ContactStateEnum, ContactStateEnum::OPEN, ct_string("OPEN")>,
                                  std_utils::CompileTimeKey<ContactStateEnum, ContactStateEnum::CLOSED, ct_string("CLOSED")>,
                                  std_utils::CompileTimeKey<ContactStateEnum, ContactStateEnum::SLIPPING, ct_string("SLIPPING")>>;
  static constexpr auto contactStateKeys = ct_contactStateKeys::getKeysArray();

  #define SMB_COORDINATE_FRAMES_NAMES WORLD, BASE, BODY
  CONSECUTIVE_ENUM_FROM_LIST(CoordinateFrameEnum, SMB_COORDINATE_FRAMES_NAMES);
  using ct_coordinateFrameKeys =
      std_utils::CompileTimeKeys<CoordinateFrameEnum, 
                                  std_utils::CompileTimeKey<CoordinateFrameEnum, CoordinateFrameEnum::WORLD, ct_string("world")>,
                                  std_utils::CompileTimeKey<CoordinateFrameEnum, CoordinateFrameEnum::BASE, ct_string("base")>,
                                  std_utils::CompileTimeKey<CoordinateFrameEnum, CoordinateFrameEnum::BODY, ct_string("body")>>;
  static constexpr auto coordinateFrameKeys = ct_coordinateFrameKeys::getKeysArray();

//TODO keep these?
//  #define SMB_FRAME_TRANSFORM_NAMES MapToOdom, MapGaToOdom
//  CONSECUTIVE_ENUM_FROM_LIST(FrameTransformEnum, SMB_FRAME_TRANSFORM_NAMES);
//  using ct_frameTransformKeys =
//      std_utils::CompileTimeKeys<FrameTransformEnum,
//                                  std_utils::CompileTimeKey<FrameTransformEnum, FrameTransformEnum::MapToOdom, ct_string("map_to_odom")>,
//                                  std_utils::CompileTimeKey<FrameTransformEnum, FrameTransformEnum::MapGaToOdom, ct_string("map_ga_to_odom")>>;
//  static constexpr auto frameTransformKeys = ct_frameTransformKeys::getKeysArray();



  // ----- Maps ----- //

//   template<ActuatorEnum actuator, LimbEnum limb>
   using mapActuatorEnumToLimbEnum =
   std_utils::CompileTimeMap<ActuatorEnum, LimbEnum>;

//   template<JointEnum joint, JointNodeEnum jointNode>
   using mapJointEnumToJointNodeEnum =
   std_utils::CompileTimeMap<JointEnum, JointNodeEnum>;

  template<ActuatorEnum actuator, ActuatorNodeEnum actuatorNode>
  using actuatorToAcutatorNodeKV = std_utils::KeyValuePair<ActuatorEnum, ActuatorNodeEnum, actuator, actuatorNode>;
  using mapActuatorEnumToActuatorNodeEnum =
  std_utils::CompileTimeMap<ActuatorEnum, ActuatorNodeEnum,
    actuatorToAcutatorNodeKV<ActuatorEnum::LF_WHEEL,   ActuatorNodeEnum::WHEEL>,
    actuatorToAcutatorNodeKV<ActuatorEnum::RF_WHEEL,   ActuatorNodeEnum::WHEEL>,
    actuatorToAcutatorNodeKV<ActuatorEnum::LH_WHEEL,   ActuatorNodeEnum::WHEEL>,
    actuatorToAcutatorNodeKV<ActuatorEnum::RH_WHEEL,   ActuatorNodeEnum::WHEEL>>;

  template<SmbActuatorEnum smbActuatorEnum, ActuatorEnum actuatorEnum>
  using smbActuatorToActuatorKV = std_utils::KeyValuePair<SmbActuatorEnum, ActuatorEnum, smbActuatorEnum, actuatorEnum>;
  using mapSmbActuatorEnumToActuatorEnum =
  std_utils::CompileTimeMap<SmbActuatorEnum, ActuatorEnum,
    smbActuatorToActuatorKV<SmbActuatorEnum::LF_WHEEL, ActuatorEnum::LF_WHEEL>,
    smbActuatorToActuatorKV<SmbActuatorEnum::RF_WHEEL, ActuatorEnum::RF_WHEEL>,
    smbActuatorToActuatorKV<SmbActuatorEnum::LH_WHEEL, ActuatorEnum::LH_WHEEL>,
    smbActuatorToActuatorKV<SmbActuatorEnum::RH_WHEEL, ActuatorEnum::RH_WHEEL>>;

//   template<ActuatorEnum actuator, JointEnum joint>
   using mapActuatorEnumToJointEnum =
   std_utils::CompileTimeMap<ActuatorEnum, JointEnum>;

//   template<JointEnum joint, ActuatorEnum actuator>
   using mapJointEnumToActuatorEnum =
   std_utils::CompileTimeMap<JointEnum, ActuatorEnum>;

  template<BodyEnum body, BranchEnum branch>
  using bodyToBranchKV = std_utils::KeyValuePair<BodyEnum, BranchEnum, body, branch>;
  using mapBodyEnumToBranchEnum =
  std_utils::CompileTimeMap<BodyEnum, BranchEnum,
    bodyToBranchKV<BodyEnum::BASE,     BranchEnum::BASE> >;

  template<BodyEnum body, BodyNodeEnum bodyNode>
  using bodyToBodyNodeKV = std_utils::KeyValuePair<BodyEnum, BodyNodeEnum, body, bodyNode>;
  using mapBodyEnumToBodyNodeEnum =
  std_utils::CompileTimeMap<BodyEnum, BodyNodeEnum,
    bodyToBodyNodeKV<BodyEnum::BASE,     BodyNodeEnum::BASE> >;

  template<BranchEnum branch, LimbEnum limb>
  using branchEnumToLimbEnumKV = std_utils::KeyValuePair<BranchEnum, LimbEnum, branch, limb>;
  using mapBranchEnumToLimbEnum =
  std_utils::CompileTimeMap<BranchEnum, LimbEnum,
//    branchEnumToLimbEnumKV<BranchEnum::DUMMY, LimbEnum::DUMMY>,
    branchEnumToLimbEnumKV<BranchEnum::BASE, LimbEnum::DUMMY>>;

  template<ContactEnum contact, BodyEnum body>
  using contactToBodyKV = std_utils::KeyValuePair<ContactEnum, BodyEnum, contact, body>;
  using mapContactEnumToBodyEnum =
  std_utils::CompileTimeMap<ContactEnum, BodyEnum,
      contactToBodyKV<ContactEnum::BASE, BodyEnum::BASE> >;

  template <BranchEnum branch, BodyEnum body>
  using branchToBodyKV = std_utils::KeyValuePair<BranchEnum, BodyEnum, branch, body>;

  using mapBranchToStartBody =
      std_utils::CompileTimeMap<BranchEnum, BodyEnum, 
                                branchToBodyKV<BranchEnum::BASE, BodyEnum::BASE>>;

  using mapBranchToEndBody =
      std_utils::CompileTimeMap<BranchEnum, BodyEnum,
                                branchToBodyKV<BranchEnum::BASE, BodyEnum::BASE>>;

        //TODO probably remove these commented lines
//   template <LimbEnum limb, unsigned int uint>
//   using limbToNDofKV = std_utils::KeyValuePair<LimbEnum , unsigned int, limb, uint>;
   using mapLimbToNDof =
   std_utils::CompileTimeMap< SmbTopology::LimbEnum , unsigned int>;


   using mapLimbToStartIndexInJ =
   std_utils::CompileTimeMap< LimbEnum , unsigned int>;

  template <BranchEnum branch, unsigned int uint>
  using branchToUIntKV = std_utils::KeyValuePair<BranchEnum, unsigned int, branch, uint>;

  using mapBranchToStartIndexInU =
  std_utils::CompileTimeMap< BranchEnum , unsigned int,
                             branchToUIntKV<BranchEnum::BASE,    0u>>;

};

} /* namespace smb_description */
