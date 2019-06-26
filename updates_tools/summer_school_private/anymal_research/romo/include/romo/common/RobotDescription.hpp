/*!
 * @file     RobotDescription.hpp
 * @author   Gabriel Hottiger, Dario Bellicoso
 * @date     Oct, 2017
 */

#pragma once

// romo
#include "romo/common/enum_maps.hpp"
#include "romo/common/enum_keys.hpp"

// std utils
#include <std_utils/std_utils.hpp>

// Eigen
#include <Eigen/Core>

// STL
#include <type_traits>
#include <array>

namespace romo {

/**
 * @brief Helper type to define a ConcreteDescription
 * @tparam ConcreteDefinitions_ Concrete Definitions implementing the static getters
 * @tparam ConcreteTopology_    Concrete Toplogy defining all enums, keys and mappings between them
 */
template < typename ConcreteDefinitions_, typename ConcreteTopology_ >
struct ConcreteDescription {
  using ConcreteDefinitions = ConcreteDefinitions_;
  using ConcreteTopology = ConcreteTopology_;
};

/**
 * @brief Static base class for all robot descriptions:
 *
 * @tparam ConcreteDescription_
 *    ConcreteDescription_::ConcreteTopology should implement:
 *      Consecutive Enums: (std_utils/ConsecutiveEnum.hpp)
 *            -> LimbEnum
 *            -> BranchEnum
 *            -> JointNodeEnum
 *            -> JointEnum
 *            -> ActuatorNodeEnum
 *            -> ActuatorEnum
 *            -> BodyNodeEnum
 *            -> BodyEnum (MUST contain all movable bodies, can contain additional fixed bodies)
 *            -> GeneralizedCoordinatesEnum
 *            -> GeneralizedVelocitiesEnum
 *            -> ContactEnum
 *            -> ContactStateEnum
 *            -> CoordinateFrameEnum
 *
 *      Key Arrays: (std_utils/containers/KeyArray.hpp)
 *            -> limbKeys
 *            -> branchKeys
 *            -> jointNodeKeys
 *            -> jointKeys
 *            -> actuatorNodeKeys
 *            -> actuatorKeys
 *            -> bodyNodeKeys
 *            -> bodyKeys
 *            -> generalizedCoordinateKeys
 *            -> generalizedVelocityKeys
 *            -> contactKeys
 *            -> contactStateKeys
 *            -> coordinateFrameKeys
 *
 *      Compile Time Maps: (std_utils/containers/CompileTimeMap.hpp)
 *            -> mapBranchEnumToLimbEnum
 *            -> mapJointEnumToActuatorEnum
 *            -> mapActuatorEnumToLimbEnum
 *            -> mapJointEnumToJointNodeEnum
 *            -> mapActuatorEnumToActuatorNodeEnum
 *            -> mapBodyEnumToBranchEnum
 *            -> mapBodyEnumToBodyNodeEnum
 *            -> mapContactEnumToBodyEnum
 *            -> mapLimbToStartJoint
 *
 *    ConcreteDescription_::ConcreteDescription should implement:
 *            ->getNumLegsImpl()
 *            ->getNumArmsImpl()
 *            ->getNumDofImpl()
 *            ->getNumDofLimbImpl(LimbEnum limb)
 *            ->getBranchStartIndexInUImpl(BranchEnum branch)
 *            ->getLimbStartIndexInJImpl(LimbEnum limb)
 *            ->getLimbStartBodyImpl(LimbEnum limb)
 *            ->getLimbEndBodyImpl(LimbEnum limb)
 *            ->getLimbStartJointImpl(LimbEnum limb)
 *            ->getBaseGeneralizedCoordinatesDimensionImpl
 *            ->getBaseGeneralizedVelocitiesDimensionImpl()
 *
 *      The following functions can be implemented if they are meaningful:
 *            ->getNumDofLimbImpl()
 *            ->getWheelDiameterImpl()
 *
 */
template < typename ConcreteDescription_ >
struct RobotDescription {

  //! Delete constructor, class consists only of static members
  RobotDescription() = delete;

  //! Expose Types
  using ConcreteDefinitions = typename ConcreteDescription_::ConcreteDefinitions;
  using ConcreteTopology = typename ConcreteDescription_::ConcreteTopology;

  //! Expose Enum Types
  using LimbEnum = typename ConcreteTopology::LimbEnum;
  using BranchEnum = typename ConcreteTopology::BranchEnum;
  using JointNodeEnum = typename ConcreteTopology::JointNodeEnum;
  using JointEnum = typename ConcreteTopology::JointEnum;
  using ActuatorNodeEnum = typename ConcreteTopology::ActuatorNodeEnum;
  using ActuatorEnum = typename ConcreteTopology::ActuatorEnum;
  using BodyNodeEnum = typename ConcreteTopology::BodyNodeEnum;
  using BodyEnum = typename ConcreteTopology::BodyEnum;
  using GeneralizedCoordinatesEnum = typename ConcreteTopology::GeneralizedCoordinatesEnum;
  using GeneralizedVelocitiesEnum = typename ConcreteTopology::GeneralizedVelocitiesEnum;
  using ContactEnum = typename ConcreteTopology::ContactEnum;
  using ContactStateEnum = typename ConcreteTopology::ContactStateEnum;
  using CoordinateFrameEnum = typename ConcreteTopology::CoordinateFrameEnum;

  //! Expose Map Types
  using mapBranchEnumToLimbEnum = typename ConcreteTopology::mapBranchEnumToLimbEnum;
  using mapLimbEnumToBranchEnum = std_utils::ctm_invert_t<LimbEnum, BranchEnum, mapBranchEnumToLimbEnum>;
  using mapJointEnumToActuatorEnum = typename ConcreteTopology::mapJointEnumToActuatorEnum;
  using mapJointEnumToJointNodeEnum = typename ConcreteTopology::mapJointEnumToJointNodeEnum;
  using mapActuatorEnumToJointEnum = std_utils::ctm_invert_t<ActuatorEnum, JointEnum, mapJointEnumToActuatorEnum>;
  using mapActuatorEnumToLimbEnum = typename ConcreteTopology::mapActuatorEnumToLimbEnum;
  using mapActuatorEnumToActuatorNodeEnum = typename ConcreteTopology::mapActuatorEnumToActuatorNodeEnum;
  using mapBodyEnumToBranchEnum = typename ConcreteTopology::mapBodyEnumToBranchEnum;
  using mapBodyEnumToBodyNodeEnum = typename ConcreteTopology::mapBodyEnumToBodyNodeEnum;
  using mapContactEnumToBodyEnum = typename ConcreteTopology::mapContactEnumToBodyEnum;
  using mapContactEnumToBodyNodeEnum = std_utils::ctm_transform_t<ContactEnum, BodyNodeEnum,
                                                                  mapContactEnumToBodyEnum,
                                                                  void, mapBodyEnumToBodyNodeEnum>;
  using mapContactEnumToBranchEnum = std_utils::ctm_transform_t<ContactEnum, BranchEnum,
                                                                mapContactEnumToBodyEnum,
                                                                void, mapBodyEnumToBranchEnum>;
  using mapContactEnumToLimbEnum = std_utils::ctm_transform_t<ContactEnum, LimbEnum,
                                                              mapContactEnumToBranchEnum,
                                                              void, mapBranchEnumToLimbEnum>;
  using mapBodyEnumToLimbEnum = std_utils::ctm_transform_t<BodyEnum, LimbEnum,
                                                           std_utils::ctm_erase_t<BodyEnum, BranchEnum, mapBodyEnumToBranchEnum, BodyEnum::BASE>,
                                                           void, mapBranchEnumToLimbEnum>;
  using mapActuatorEnumToBranchEnum = std_utils::ctm_transform_t<ActuatorEnum, BranchEnum,
                                                                 mapActuatorEnumToLimbEnum,
                                                                 void, mapLimbEnumToBranchEnum>;
  using mapActuatorEnumToJointNodeEnum = std_utils::ctm_transform_t<ActuatorEnum, JointNodeEnum,
                                                                    mapActuatorEnumToJointEnum ,
                                                                    void, mapJointEnumToJointNodeEnum>;
  using mapJointEnumToActuatorNodeEnum = std_utils::ctm_transform_t<JointEnum, ActuatorNodeEnum,
                                                                    mapJointEnumToActuatorEnum,
                                                                    void, mapActuatorEnumToActuatorNodeEnum>;
  using mapJointEnumToLimbEnum = std_utils::ctm_transform_t<JointEnum, LimbEnum,
                                                            mapActuatorEnumToLimbEnum,
                                                            mapActuatorEnumToJointEnum, void>;
  using mapJointEnumToBranchEnum = std_utils::ctm_transform_t<JointEnum, BranchEnum,
                                                              mapJointEnumToLimbEnum,
                                                              void, mapLimbEnumToBranchEnum>;

  /**
   * @brief Get keys for enum Type Enum_
   * @tparam Enum_ enum type
   * @return keys
   */
  template <typename Enum_>
  inline static constexpr const std_utils::KeyArray<Enum_> getKeys() {
    return internal::KeysHelper<ConcreteTopology, Enum_>::keys();
  }

  //! Non-templated key getters
  inline static constexpr const std_utils::KeyArray<LimbEnum> getLimbKeys()                 { return getKeys<LimbEnum>(); }
  inline static constexpr const std_utils::KeyArray<BranchEnum> getBranchKeys()             { return getKeys<BranchEnum>(); }
  inline static constexpr const std_utils::KeyArray<JointNodeEnum> getJointNodeKeys()       { return getKeys<JointNodeEnum>(); }
  inline static constexpr const std_utils::KeyArray<JointEnum> getJointKeys()               { return getKeys<JointEnum>(); }
  inline static constexpr const std_utils::KeyArray<ActuatorNodeEnum> getActuatorNodeKeys() { return getKeys<ActuatorNodeEnum>(); }
  inline static constexpr const std_utils::KeyArray<ActuatorEnum> getActuatorKeys()         { return getKeys<ActuatorEnum>(); }
  inline static constexpr const std_utils::KeyArray<BodyNodeEnum > getBodyNodeKeys()        { return getKeys<BodyNodeEnum>(); }
  inline static constexpr const std_utils::KeyArray<BodyEnum> getBodyKeys()                 { return getKeys<BodyEnum>(); }
  inline static constexpr const std_utils::KeyArray<GeneralizedCoordinatesEnum> getGeneralizedCoordinateKeys() { return getKeys<GeneralizedCoordinatesEnum>(); }
  inline static constexpr const std_utils::KeyArray<GeneralizedVelocitiesEnum > getGeneralizedVelocityKeys()   { return getKeys<GeneralizedVelocitiesEnum>(); }
  inline static constexpr const std_utils::KeyArray<ContactEnum> getContactKeys()                 { return getKeys<ContactEnum>(); }
  inline static constexpr const std_utils::KeyArray<ContactStateEnum > getContactStateKeys()      { return getKeys<ContactStateEnum>(); }
  inline static constexpr const std_utils::KeyArray<CoordinateFrameEnum> getCoordinateFrameKeys() { return getKeys<CoordinateFrameEnum>(); }

  //! @return number of limbs
  inline static constexpr unsigned int getNumLimbs() { return static_cast<unsigned int>(LimbEnum::SIZE); }

  //! @return number of legs
  inline static constexpr unsigned int getNumLegs() { return ConcreteDefinitions::getNumLegsImpl(); }

  //! @return number of arms
  inline static constexpr unsigned int getNumArms() { return ConcreteDefinitions::getNumArmsImpl(); }

  //! @return number of degrees of freedom
  inline static constexpr unsigned int getNumDof() { return ConcreteDefinitions::getNumDofImpl(); }

  //! @return number of degrees of freedom of limb
  inline static constexpr unsigned int getNumDofLimb(LimbEnum limb) { return ConcreteDefinitions::getNumDofLimbImpl(limb); }

  /* @return number of degrees of freedom of a limb
   * @note Assuming all limbs have the same number of dof's, don't implement this function if this is not the case!
   */
  inline static constexpr unsigned int getNumDofLimb() { return ConcreteDefinitions::getNumDofLimbImpl(); }

  //! @return start index of branch in the generalized velocity vector (e.g. to get relevant block in jacobians)
  inline static constexpr unsigned int getBranchStartIndexInU(BranchEnum branch) { return ConcreteDefinitions::getBranchStartIndexInUImpl(branch); }

  //! @return start index of limb in the joints vector (e.g. to get relevant block in joint states)
  inline static constexpr unsigned int getLimbStartIndexInJ(LimbEnum limb) { return ConcreteDefinitions::getLimbStartIndexInJImpl(limb); }

  //! @return start body of branch
  inline static constexpr BodyEnum getBranchStartBody(BranchEnum branch) { return ConcreteDefinitions::getBranchStartBodyImpl(branch); }

  //! @return end body of limb
  inline static constexpr BodyEnum getBranchEndBody(BranchEnum branch) { return ConcreteDefinitions::getBranchEndBodyImpl(branch); }

  //! @return start joint of limb
  inline static constexpr JointEnum getLimbStartJoint(LimbEnum limb) { return ConcreteDefinitions::getLimbStartJointImpl(limb); }

  //! @return number of actuators
  inline static constexpr unsigned int getActuatorsDimension() { return static_cast<unsigned int>(ActuatorEnum::SIZE); }

  //! @return number of joints
  inline static constexpr unsigned int getJointsDimension() { return static_cast<unsigned int>(JointEnum::SIZE); }

  //! @return number of branches
  inline static constexpr unsigned int getBranchesDimension() { return static_cast<unsigned int>(BranchEnum::SIZE); }

  //! @return number of generalized coordinates
  inline static constexpr unsigned int getGeneralizedCoordinatesDimension() { return static_cast<unsigned int>(GeneralizedCoordinatesEnum::SIZE); }

  //! @return number of generalized velocities
  inline static constexpr unsigned int getGeneralizedVelocitiesDimension() { return static_cast<unsigned int>(GeneralizedVelocitiesEnum::SIZE); }

  //! @return number of generalized accelerations
  inline static constexpr unsigned int getGeneralizedAccelerationsDimension() { return static_cast<unsigned int>(GeneralizedVelocitiesEnum::SIZE); }

  //! @return number of generalized coordinates of the base
  inline static constexpr unsigned int getBaseGeneralizedCoordinatesDimension() { return ConcreteDefinitions::getBaseGeneralizedCoordinatesDimensionImpl(); }

  //! @return number of generalized velocities of the base
  inline static constexpr unsigned int getBaseGeneralizedVelocitiesDimension() { return ConcreteDefinitions::getBaseGeneralizedVelocitiesDimensionImpl(); }

  //! @return wheel diameter
  inline static constexpr double getWheelDiameter(LimbEnum limb) { return ConcreteDefinitions::getWheelDiameterImpl(); }

  //! @return index of the endeffector in the limb joints list
  inline static constexpr unsigned int getWheelIndexInJ(LimbEnum limb) { return ConcreteDefinitions::getWheelIndexInJImpl(limb); }


  //! Cartesian helper functions
  inline static constexpr unsigned int getNumTranslationalDof() { return 3; }
  inline static constexpr unsigned int getNumRotationalDof() { return 3; }
  inline static constexpr unsigned int getNumSpatialDof() { return getNumTranslationalDof() + getNumRotationalDof(); }

  //! Typedefs
  using JointVector = Eigen::Matrix<double, getJointsDimension(), 1>;
  using MassMatrix = Eigen::Matrix<double, getNumDof(), getNumDof()>;
  using NonlinearEffects = Eigen::Matrix<double, getNumDof(), 1>;
  using GravityTerms = Eigen::Matrix<double, getNumDof(), 1>;
  using JacobianSpatial = Eigen::Matrix<double, getNumSpatialDof(), getNumDof()>;
  using JacobianTranslation = Eigen::Matrix<double, getNumTranslationalDof(), getNumDof()>;
  using JacobianRotation = Eigen::Matrix<double, getNumRotationalDof(), getNumDof()>;
  using JacobianTranslationTimeDerivative = JacobianTranslation;
  using JacobianRotationTimeDerivative  = JacobianRotation;

  /**
   * @brief Map enum to itself
   * @tparam ToEnum_    Enum Type
   * @tparam FromEnum_  Identical Enum Type as ToEnum_, forced by enable_if
   * @param e           Enum value to map
   * @return            e, input=output behavior
   */
  template <typename ToEnum_, typename FromEnum_>
  inline static constexpr ToEnum_ mapEnums( FromEnum_ e, typename std::enable_if<std::is_same<ToEnum_, FromEnum_>::value>::type* = 0) {
    return e;
  }

  /**
   * @brief Map enum value of type FromEnum_ to corresponding value of enum type ToEnum_
   * @tparam ToEnum_    Output enum type
   * @tparam FromEnum_  Input enum type
   * @param e           Enum value to map
   * @return            mapped value of enum type ToEnum_
   */
  template <typename ToEnum_, typename FromEnum_>
  inline static constexpr ToEnum_ mapEnums( FromEnum_ e, typename std::enable_if<!std::is_same<ToEnum_, FromEnum_>::value>::type* = 0) {
    return internal::MapEnumHelper<FromEnum_, ToEnum_, ConcreteTopology >::map(e);
  }

  /**
   * @brief Maps a Key Id to a Key Enum, if only one argument is given it maps to itself.
   * @tparam FromEnum_ Enum type corresponding to the id
   * @tparam ToEnum_   Mapped enum type ( default: FromEnum_ )
   * @param id         Key Id to be mapped to an enum
   * @return           mapped key enum
   */
  template <typename FromEnum_, typename ToEnum_ = FromEnum_>
  inline static constexpr ToEnum_ mapKeyIdToKeyEnum( typename std_utils::KeyArray<FromEnum_>::IdType id) {
    return mapEnums<ToEnum_>(static_cast<FromEnum_>(id));
  }

  /**
   * @brief Maps a Key Id to a Key Name, if only one argument is given it maps to itself.
   * @tparam FromEnum_ Enum type corresponding to the id
   * @tparam ToEnum_   Mapped enum type ( default: FromEnum_ )
   * @param id         Key Id to be mapped to a name
   * @return            mapped key name
   */
  template <typename FromEnum_, typename ToEnum_ = FromEnum_>
  inline static constexpr const char * mapKeyIdToKeyName( typename std_utils::KeyArray<FromEnum_>::IdType id) {
    return getKeys<ToEnum_>()[mapEnums<ToEnum_>(static_cast<FromEnum_>(id))].getName();
  }

  /**
   * @brief Maps a Key Id to a Key Id, if only one argument is given it maps to itself.
   * @tparam FromEnum_ Enum type corresponding to the id
   * @tparam ToEnum_   Mapped enum type ( default: FromEnum_ )
   * @param id         Key Id to be mapped to another id
   * @return           mapped key id
   */
  template <typename FromEnum_, typename ToEnum_ = FromEnum_>
  inline static constexpr typename std_utils::KeyArray<ToEnum_>::IdType mapKeyIdToKeyId( typename std_utils::KeyArray<FromEnum_>::IdType id) {
    return getKeys<ToEnum_>()[mapEnums<ToEnum_>(static_cast<FromEnum_>(id))].getId();
  }

  /**
   * @brief Maps a Key Enum to another Key Enum, if only one argument is given it maps to itself.
   *        This is save since the compiler will complain if the user specifies a wrong single type.
   * @tparam FromEnum_ Enum type corresponding to the e
   * @tparam ToEnum_   Mapped enum type ( default: FromEnum_ )
   * @param e          Key enum to be mapped to another enum
   * @return           mapped key enum
   */
  template <typename FromEnum_, typename ToEnum_ = FromEnum_>
  inline static constexpr ToEnum_ mapKeyEnumToKeyEnum( FromEnum_ e ) {
    return mapEnums<ToEnum_>(e);
  }

  /**
   * @brief Maps a Key Enum to a Key name, if only one argument is given it maps to itself.
   *        This is save since the compiler will complain if the user specifies a wrong single type.
   * @tparam FromEnum_ Enum type corresponding to the e
   * @tparam ToEnum_   Mapped enum type ( default: FromEnum_ )
   * @param e          Key enum to be mapped to a key name
   * @return           mapped key name
   */
  template <typename FromEnum_, typename ToEnum_ = FromEnum_>
  inline static constexpr const char * mapKeyEnumToKeyName( FromEnum_ e ) {
    return getKeys<ToEnum_>()[mapEnums<ToEnum_>(e)].getName();
  }

  /**
   * @brief Maps a Key Enum to a Key id, if only one argument is given it maps to itself.
   *        This is save since the compiler will complain if the user specifies a wrong single type.
   * @tparam FromEnum_ Enum type corresponding to the e
   * @tparam ToEnum_   Mapped enum type ( default: FromEnum_ )
   * @param e          Key enum to be mapped to a key id
   * @return           mapped key id
   */
  template <typename FromEnum_, typename ToEnum_ = FromEnum_>
  inline static constexpr typename std_utils::KeyArray<ToEnum_>::IdType mapKeyEnumToKeyId( FromEnum_ e ) {
    return getKeys<ToEnum_>()[mapEnums<ToEnum_>(e)].getId();
  }

  /**
   * @brief Maps a Key Name to a Key Enum, if only one argument is given it maps to itself.
   * @tparam FromEnum_ Enum type corresponding to Key Name
   * @tparam ToEnum_   Mapped enum type ( default: FromEnum_ )
   * @param e          Key name to be mapped to a key enum
   * @return           mapped key enum
   */
  template <typename FromEnum_, typename ToEnum_ = FromEnum_>
  inline static const ToEnum_ mapKeyNameToKeyEnum( const std::string & name ) {
    return mapEnums<ToEnum_>(getKeys<FromEnum_>().atName(name).getEnum());
  }

  /**
   * @brief Maps a Key Name to a Key Name, if only one argument is given it maps to itself.
   * @tparam FromEnum_ Enum type corresponding to Key Name
   * @tparam ToEnum_   Mapped enum type ( default: FromEnum_ )
   * @param e          Key name to be mapped to a key name
   * @return           mapped key name
   */
  template <typename FromEnum_, typename ToEnum_ = FromEnum_>
  inline static const char * mapKeyNameToKeyName( const std::string & name ) {
    return getKeys<ToEnum_>()[mapEnums<ToEnum_>(getKeys<FromEnum_>().atName(name).getEnum())].getName();
  }

  /**
   * @brief Maps a Key Name to a Key Id, if only one argument is given it maps to itself.
   * @tparam FromEnum_ Enum type corresponding to Key Name
   * @tparam ToEnum_   Mapped enum type ( default: FromEnum_ )
   * @param e          Key name to be mapped to a key id
   * @return           mapped key id
   */
  template <typename FromEnum_, typename ToEnum_ = FromEnum_>
  inline static const typename std_utils::KeyArray<ToEnum_>::IdType mapKeyNameToKeyId( const std::string & name ) {
    return getKeys<ToEnum_>()[mapEnums<ToEnum_>(getKeys<FromEnum_>().atName(name).getEnum())].getId();
  }
};

}  // namespace romo
