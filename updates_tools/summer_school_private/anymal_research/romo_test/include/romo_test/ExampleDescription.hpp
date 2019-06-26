/*!
 * @file     ExampleDescription.hpp
 * @author   Gabriel Hottiger
 * @date     Nov, 2017
 */

#pragma once

// std_utils
#include <std_utils/std_utils.hpp>

// romo
#include <romo/common/RobotDescription.hpp>

namespace romo_test {

struct ExampleTopology {

  // Example Topology for a 6-dof floating base with 3-dof Arm

  // Use the CONSECUTIVE_ENUM helper macro, to make sure the enums are consecutive and SIZE-terminated
  CONSECUTIVE_ENUM(BranchEnum, BASE, ARM);
  /* This expands to:
   * enum class BranchEnum : unsigned int {
   *    BASE = 0,
   *    ARM,
   *    SIZE
   * };
   */

  // To relate the enums to string build up a key array
  static constexpr std_utils::KeyArray<BranchEnum> branchKeys { { std_utils::make_key(BranchEnum::BASE, "BASE"),
                                                                  std_utils::make_key(BranchEnum::ARM,  "ARM") } };

  /* Repeat this procedure for the following enums
   * - BranchEnum
   * - LimbEnum
   * - JointEnum
   * - ActuatorEnum
   * - BodyNodeEnum
   * - BodyEnum
   * - GeneralizedCoordinatesEnum
   * - GeneralizedVelocitiesEnum
   * - ContactEnum
   * - ContactStateEnum
   * - CoordinateFrameEnum
   */
  CONSECUTIVE_ENUM(LimbEnum, ARM);
  static constexpr std_utils::KeyArray<LimbEnum> limbKeys { { std_utils::make_key(LimbEnum::ARM,   "ARM") } };

  CONSECUTIVE_ENUM(JointNodeEnum, J1, J2, J3);
  static constexpr std_utils::KeyArray<JointNodeEnum> jointNodeKeys { { std_utils::make_key(JointNodeEnum::J1,    "Joint1"),
                                                                        std_utils::make_key(JointNodeEnum::J2,    "Joint2"),
                                                                        std_utils::make_key(JointNodeEnum::J3,    "Joint3") } };

  CONSECUTIVE_ENUM(JointEnum, ARM_J1, ARM_J2, ARM_J3);
  static constexpr std_utils::KeyArray<JointEnum> jointKeys { { std_utils::make_key(JointEnum::ARM_J1, "ARM_Joint1"),
                                                                std_utils::make_key(JointEnum::ARM_J2, "ARM_Joint2"),
                                                                std_utils::make_key(JointEnum::ARM_J3, "ARM_Joint3") } };

  CONSECUTIVE_ENUM(ActuatorNodeEnum, M1, M2, M3);
  static constexpr std_utils::KeyArray<ActuatorNodeEnum> actuatorNodeKeys { { std_utils::make_key(ActuatorNodeEnum::M1,    "Motor1"),
                                                                              std_utils::make_key(ActuatorNodeEnum::M2,    "Motor2"),
                                                                              std_utils::make_key(ActuatorNodeEnum::M3,    "Motor3") } };

  CONSECUTIVE_ENUM(ActuatorEnum, ARM_M1, ARM_M2, ARM_M3);
  static constexpr std_utils::KeyArray<ActuatorEnum> actuatorKeys { { std_utils::make_key(ActuatorEnum::ARM_M1, "ARM_Motor1"),
                                                                      std_utils::make_key(ActuatorEnum::ARM_M2, "ARM_Motor2"),
                                                                      std_utils::make_key(ActuatorEnum::ARM_M3, "ARM_Motor3") } };

  CONSECUTIVE_ENUM(BodyNodeEnum, BASE, UPPER_ARM, FORE_ARM, TOOL);
  static constexpr std_utils::KeyArray<BodyNodeEnum> bodyNodeKeys { { std_utils::make_key(BodyNodeEnum::BASE,        "BASE"),
                                                                      std_utils::make_key(BodyNodeEnum::UPPER_ARM,   "UPPER_ARM"),
                                                                      std_utils::make_key(BodyNodeEnum::FORE_ARM,    "FORE_ARM"),
                                                                      std_utils::make_key(BodyNodeEnum::TOOL,        "TOOL") } };

  CONSECUTIVE_ENUM(BodyEnum, BASE, UPPER_ARM, FORE_ARM, TOOL);
  static constexpr std_utils::KeyArray<BodyEnum> bodyKeys { { std_utils::make_key(BodyEnum::BASE,       "BASE"),
                                                              std_utils::make_key(BodyEnum::UPPER_ARM,  "UPPER_ARM"),
                                                              std_utils::make_key(BodyEnum::FORE_ARM,   "FORE_ARM"),
                                                              std_utils::make_key(BodyEnum::TOOL,       "TOOL") } };

  CONSECUTIVE_ENUM(GeneralizedCoordinatesEnum, X, Y, Z, Q_W, Q_X, Q_Y, Q_Z, J1, J2, J3);
  static constexpr std_utils::KeyArray<GeneralizedCoordinatesEnum> generalizedCoordinateKeys { {  std_utils::make_key(GeneralizedCoordinatesEnum::X,        "X"),
                                                                                                  std_utils::make_key(GeneralizedCoordinatesEnum::Y,        "Y"),
                                                                                                  std_utils::make_key(GeneralizedCoordinatesEnum::Z,        "Z"),
                                                                                                  std_utils::make_key(GeneralizedCoordinatesEnum::Q_W,      "Q_W"),
                                                                                                  std_utils::make_key(GeneralizedCoordinatesEnum::Q_X,      "Q_X"),
                                                                                                  std_utils::make_key(GeneralizedCoordinatesEnum::Q_Y,      "Q_Y"),
                                                                                                  std_utils::make_key(GeneralizedCoordinatesEnum::Q_Z,      "Q_Z"),
                                                                                                  std_utils::make_key(GeneralizedCoordinatesEnum::J1,       "J1"),
                                                                                                  std_utils::make_key(GeneralizedCoordinatesEnum::J2,       "J2"),
                                                                                                  std_utils::make_key(GeneralizedCoordinatesEnum::J3,       "J3") } };


  CONSECUTIVE_ENUM(GeneralizedVelocitiesEnum, L_X, L_Y, L_Z, A_X, A_Y, A_Z, J1, J2, J3);
  static constexpr std_utils::KeyArray<GeneralizedVelocitiesEnum> generalizedVelocityKeys { { std_utils::make_key(GeneralizedVelocitiesEnum::L_X,      "L_X"),
                                                                                              std_utils::make_key(GeneralizedVelocitiesEnum::L_Y,      "L_Y"),
                                                                                              std_utils::make_key(GeneralizedVelocitiesEnum::L_Z,      "L_Z"),
                                                                                              std_utils::make_key(GeneralizedVelocitiesEnum::A_X,      "A_X"),
                                                                                              std_utils::make_key(GeneralizedVelocitiesEnum::A_Y,      "A_Y"),
                                                                                              std_utils::make_key(GeneralizedVelocitiesEnum::A_Z,      "A_Z"),
                                                                                              std_utils::make_key(GeneralizedVelocitiesEnum::J1,       "J1"),
                                                                                              std_utils::make_key(GeneralizedVelocitiesEnum::J2,       "J2"),
                                                                                              std_utils::make_key(GeneralizedVelocitiesEnum::J3,       "J3") } };

  CONSECUTIVE_ENUM(ContactEnum, TOOL);
  static constexpr std_utils::KeyArray<ContactEnum> contactKeys { { std_utils::make_key(ContactEnum::TOOL, "TOOL") } };

  CONSECUTIVE_ENUM(ContactStateEnum, OPEN, CLOSED, SLIPPING);
  static constexpr std_utils::KeyArray<ContactStateEnum> contactStateKeys { { std_utils::make_key(ContactStateEnum::OPEN,     "OPEN"),
                                                                              std_utils::make_key(ContactStateEnum::CLOSED,   "CLOSED"),
                                                                              std_utils::make_key(ContactStateEnum::SLIPPING, "SLIPPING") } };

  CONSECUTIVE_ENUM(CoordinateFrameEnum, WORLD, BASE, BODY);
  static constexpr std_utils::KeyArray<CoordinateFrameEnum> coordinateFrameKeys { { std_utils::make_key(CoordinateFrameEnum::WORLD, "WORLD"),
                                                                                    std_utils::make_key(CoordinateFrameEnum::BASE,  "BASE"),
                                                                                    std_utils::make_key(CoordinateFrameEnum::BODY,  "BODY") } };


  /* Define the mapping between enums */
  template <ActuatorEnum actuator, LimbEnum limb>
  using actuatorToLimbKV = std_utils::KeyValuePair<ActuatorEnum , LimbEnum, actuator, limb>;
  using mapActuatorEnumToLimbEnum =  std_utils::CompileTimeMap< ActuatorEnum , LimbEnum,
                                                                actuatorToLimbKV<ActuatorEnum::ARM_M1, LimbEnum::ARM>,
                                                                actuatorToLimbKV<ActuatorEnum::ARM_M2, LimbEnum::ARM>,
                                                                actuatorToLimbKV<ActuatorEnum::ARM_M3, LimbEnum::ARM> >;

  template <ActuatorEnum actuator, ActuatorNodeEnum actuatorNode>
  using actuatorToActuatorNodeKV = std_utils::KeyValuePair<ActuatorEnum , ActuatorNodeEnum, actuator, actuatorNode>;
  using mapActuatorEnumToActuatorNodeEnum =  std_utils::CompileTimeMap<  ActuatorEnum , ActuatorNodeEnum,
                                                                      actuatorToActuatorNodeKV<ActuatorEnum::ARM_M1,   ActuatorNodeEnum::M1>,
                                                                      actuatorToActuatorNodeKV<ActuatorEnum::ARM_M2,   ActuatorNodeEnum::M2>,
                                                                      actuatorToActuatorNodeKV<ActuatorEnum::ARM_M3,   ActuatorNodeEnum::M3> >;

  template <JointEnum joint, JointNodeEnum jointNode>
  using jointToJointNodeKV = std_utils::KeyValuePair<JointEnum , JointNodeEnum, joint, jointNode>;
  using mapJointEnumToJointNodeEnum =  std_utils::CompileTimeMap<  JointEnum , JointNodeEnum,
                                                                      jointToJointNodeKV<JointEnum::ARM_J1,   JointNodeEnum::J1>,
                                                                      jointToJointNodeKV<JointEnum::ARM_J2,   JointNodeEnum::J2>,
                                                                      jointToJointNodeKV<JointEnum::ARM_J3,   JointNodeEnum::J3> >;

  template <JointEnum joint, ActuatorEnum actuator>
  using jointToActuatorKV = std_utils::KeyValuePair<JointEnum, ActuatorEnum, joint, actuator>;
  using mapJointEnumToActuatorEnum =  std_utils::CompileTimeMap<  JointEnum , ActuatorEnum,
                                                                  jointToActuatorKV<JointEnum::ARM_J1,   ActuatorEnum::ARM_M1>,
                                                                  jointToActuatorKV<JointEnum::ARM_J2,   ActuatorEnum::ARM_M2>,
                                                                  jointToActuatorKV<JointEnum::ARM_J3,   ActuatorEnum::ARM_M3> >;

  template <BodyEnum body, BranchEnum branch>
  using bodyToBranchKV = std_utils::KeyValuePair<BodyEnum , BranchEnum, body, branch>;
  using mapBodyEnumToBranchEnum =  std_utils::CompileTimeMap< BodyEnum , BranchEnum,
                                                              bodyToBranchKV<BodyEnum::BASE,        BranchEnum::BASE>,
                                                              bodyToBranchKV<BodyEnum::UPPER_ARM,   BranchEnum::ARM>,
                                                              bodyToBranchKV<BodyEnum::FORE_ARM,    BranchEnum::ARM>,
                                                              bodyToBranchKV<BodyEnum::TOOL,        BranchEnum::ARM> >;

  template <BodyEnum body, BodyNodeEnum bodyNode>
  using bodyToBodyNodeKV = std_utils::KeyValuePair<BodyEnum , BodyNodeEnum, body, bodyNode>;
  using mapBodyEnumToBodyNodeEnum =  std_utils::CompileTimeMap< BodyEnum , BodyNodeEnum,
                                                                bodyToBodyNodeKV<BodyEnum::BASE,         BodyNodeEnum::BASE>,
                                                                bodyToBodyNodeKV<BodyEnum::UPPER_ARM,    BodyNodeEnum::UPPER_ARM>,
                                                                bodyToBodyNodeKV<BodyEnum::FORE_ARM,     BodyNodeEnum::FORE_ARM>,
                                                                bodyToBodyNodeKV<BodyEnum::TOOL,         BodyNodeEnum::TOOL> >;

  template <BranchEnum branch, LimbEnum limb>
  using branchToLimbKV = std_utils::KeyValuePair<BranchEnum , LimbEnum, branch, limb>;
  using mapBranchEnumToLimbEnum =  std_utils::CompileTimeMap< BranchEnum , LimbEnum,
                                                              branchToLimbKV<BranchEnum::ARM,  LimbEnum::ARM> >;

  template <ContactEnum contact, BodyEnum body>
  using contactToBodyKV = std_utils::KeyValuePair<ContactEnum , BodyEnum , contact, body>;
  using mapContactEnumToBodyEnum =  std_utils::CompileTimeMap<  ContactEnum , BodyEnum,
                                                                contactToBodyKV<ContactEnum::TOOL,    BodyEnum::TOOL> >;
};

struct ExampleDefinitions {
  // Maps limb to nDof of limb (helper for constexpr function)
  template <ExampleTopology::LimbEnum limb, unsigned int nDof>
  using limbToNDofKV = std_utils::KeyValuePair<ExampleTopology::LimbEnum , unsigned int, limb, nDof>;
  using mapLimbToNDof =  std_utils::CompileTimeMap< ExampleTopology::LimbEnum , unsigned int,
                                                    limbToNDofKV<ExampleTopology::LimbEnum::ARM, 3u> >;

  // Maps branch to index in U of branch (helper for constexpr function)
  template <ExampleTopology::BranchEnum branch, unsigned int index>
  using branchToStartIndexInUKV = std_utils::KeyValuePair<ExampleTopology::BranchEnum , unsigned int, branch, index>;
  using mapBranchToStartIndexInU =  std_utils::CompileTimeMap< ExampleTopology::BranchEnum , unsigned int,
                                                               branchToStartIndexInUKV<ExampleTopology::BranchEnum::BASE, 0u>,
                                                               branchToStartIndexInUKV<ExampleTopology::BranchEnum::ARM,  6u>>;

  template <ExampleTopology::LimbEnum limb, ExampleTopology::BodyNodeEnum node>
  using limbToBodyNodeKV = std_utils::KeyValuePair<ExampleTopology::LimbEnum, ExampleTopology::BodyNodeEnum, limb, node>;

  using mapLimbToStartBodyNode =  std_utils::CompileTimeMap< ExampleTopology::LimbEnum , ExampleTopology::BodyNodeEnum,
                                                             limbToBodyNodeKV<ExampleTopology::LimbEnum::ARM, ExampleTopology::BodyNodeEnum::UPPER_ARM> >;
  using mapLimbToEndBodyNode =    std_utils::CompileTimeMap< ExampleTopology::LimbEnum , ExampleTopology::BodyNodeEnum,
                                                             limbToBodyNodeKV<ExampleTopology::LimbEnum::ARM, ExampleTopology::BodyNodeEnum::TOOL> >;

  inline static constexpr unsigned int getNumLegsImpl() { return 0; }
  inline static constexpr unsigned int getNumArmsImpl() { return 1; }
  inline static constexpr unsigned int getNumDofImpl() { return 9; }
  inline static constexpr unsigned int getNumDofLimbImpl(ExampleTopology::LimbEnum limb) { return mapLimbToNDof::at(limb); }
  inline static constexpr unsigned int getBranchStartIndexInUImpl(ExampleTopology::BranchEnum branch) { return mapBranchToStartIndexInU::at(branch); }
  inline static constexpr ExampleTopology::BodyNodeEnum getLimbStartBodyNodeImpl(ExampleTopology::LimbEnum limb) { return mapLimbToStartBodyNode::at(limb); }
  inline static constexpr ExampleTopology::BodyNodeEnum getLimbEndBodyNodeImpl(ExampleTopology::LimbEnum limb) { return mapLimbToEndBodyNode::at(limb); }
  inline static constexpr unsigned int getBaseGeneralizedCoordinatesDimensionImpl() { return 7; }
  inline static constexpr unsigned int getBaseGeneralizedVelocitiesDimensionImpl() { return 6; }

};

using ExampleDescription = romo::RobotDescription<romo::ConcreteDescription<ExampleDefinitions, ExampleTopology>>;

}