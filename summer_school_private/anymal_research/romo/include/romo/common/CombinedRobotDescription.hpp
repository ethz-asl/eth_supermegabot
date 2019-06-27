/*!
 * @file     CombinedRobotDescription.hpp
 * @author   Gabriel Hottiger, Dario Bellicoso
 * @date     Apr, 2018
 */

#pragma once

// std utils
#include <std_utils/std_utils.hpp>

// Eigen
#include <Eigen/Core>

// STL
#include <array>
#include <type_traits>


#define ROMO_CREATE_COMBINED_KEYS(Enum, Keys)                                                                         \
  template <typename ConcreteTopology_>                                                                               \
  struct mapSub##Enum##To##Enum {                                                                                     \
    using type = typename std::conditional<                                                                           \
        std::is_same<ConcreteTopology_, ConcreteTopology1>::value,                                                    \
        std_utils::ctm_from_subenum_t<Enum, typename ConcreteTopology1::Enum>,                                        \
        typename std::conditional<std::is_same<ConcreteTopology_, ConcreteTopology2>::value,                          \
                                  std_utils::ctm_from_subenum_t<Enum, typename ConcreteTopology2::Enum,               \
                                                                static_cast<int>(ConcreteTopology1::Enum::SIZE)>,     \
                                  void>::type>::type;                                                                 \
  };                                                                                                                  \
                                                                                                                      \
  template <typename ConcreteDescription_>                                                                            \
  using mapSub##Enum##To##Enum##_t = typename mapSub##Enum##To##Enum<ConcreteDescription_>::type;                     \
                                                                                                                      \
  using ct_##Keys =                                                                                                   \
      std_utils::ctk_combine_t<Enum, typename ConcreteTopology1::ct_##Keys, typename ConcreteTopology2::ct_##Keys,    \
                               mapSub##Enum##To##Enum##_t<ConcreteTopology1>,                                         \
                               mapSub##Enum##To##Enum##_t<ConcreteTopology2>>;                                        \
 private:                                                                                                             \
  using Keys##_type = decltype(ct_##Keys::getKeysArray());                                                            \
 public:                                                                                                              \
  static constexpr Keys##_type Keys = ct_##Keys::getKeysArray();

#define ROMO_CREATE_COMBINED_KEYS_SKIP_BASE(Enum, Keys)                                                               \
  template <typename ConcreteTopology_>                                                                               \
  struct mapSub##Enum##To##Enum {                                                                                     \
    using type = typename std::conditional<                                                                           \
        std::is_same<ConcreteTopology_, ConcreteTopology1>::value,                                                    \
        std_utils::ctm_from_subenum_t<Enum, typename ConcreteTopology1::Enum>,                                        \
        typename std::conditional<std::is_same<ConcreteTopology_, ConcreteTopology2>::value,                          \
                                  std_utils::ctm_from_subenum_t<Enum, typename ConcreteTopology2::Enum,               \
                                                                static_cast<int>(ConcreteTopology1::Enum::SIZE),1>,   \
                                  void>::type>::type;                                                                 \
  };                                                                                                                  \
                                                                                                                      \
  template <typename ConcreteDescription_>                                                                            \
  using mapSub##Enum##To##Enum##_t = typename mapSub##Enum##To##Enum<ConcreteDescription_>::type;                     \
                                                                                                                      \
  using ct_##Keys =                                                                                                   \
      std_utils::ctk_combine_t<Enum, typename ConcreteTopology1::ct_##Keys,                                           \
                               std_utils::ctk_pop_front_and_decrement_t<typename ConcreteTopology2::Enum,             \
                                                      typename ConcreteTopology2::ct_##Keys>,                         \
                               mapSub##Enum##To##Enum##_t<ConcreteTopology1>,                                         \
                               mapSub##Enum##To##Enum##_t<ConcreteTopology2>>;                                        \
 private:                                                                                                             \
  using Keys##_type = decltype(ct_##Keys::getKeysArray());                                                            \
 public:                                                                                                              \
  static constexpr Keys##_type Keys = ct_##Keys::getKeysArray();


#define ROMO_CREATE_COMBINED_MAPS(FromEnum, ToEnum, Map)                                                            \
  using Map = std_utils::ctm_combine_t<                                                                             \
      FromEnum, ToEnum, typename ConcreteTopology1::Map,                                                            \
      typename ConcreteTopology2::Map, mapSub##FromEnum##To##FromEnum##_t<ConcreteTopology1>,                       \
      mapSub##FromEnum##To##FromEnum##_t<ConcreteTopology2>, mapSub##ToEnum##To##ToEnum##_t<ConcreteTopology1>,     \
      mapSub##ToEnum##To##ToEnum##_t<ConcreteTopology2>>;

#define ROMO_CREATE_COMBINED_MAPS_SKIP_BASE(FromEnum, ToEnum, Map)                                                     \
  using Map = std_utils::ctm_combine_t<                                                                                \
      FromEnum, ToEnum,                                                                                                \
      typename ConcreteTopology1::Map,                                                                                 \
      std_utils::ctm_erase_t<typename ConcreteTopology2::FromEnum, typename ConcreteTopology2::ToEnum,                 \
                             typename ConcreteTopology2::Map, ConcreteTopology2::FromEnum::BASE>,                      \
      mapSub##FromEnum##To##FromEnum##_t<ConcreteTopology1>, mapSub##FromEnum##To##FromEnum##_t<ConcreteTopology2>,    \
      mapSub##ToEnum##To##ToEnum##_t<ConcreteTopology1>, mapSub##ToEnum##To##ToEnum##_t<ConcreteTopology2>>;

#define ROMO_CREATE_COMBINED_MAPS_CONDITIONALLY(Key, Value, Map)                                           \
 private:                                                                                                  \
  template <typename Key_, typename Value_, typename hasType1, typename hasType2>                          \
  struct Map##_helper {                                                                                    \
    using type = std_utils::CompileTimeMap<Key_, Value_>;                                                  \
    static_assert(!std::is_same<Key_, Key_>::value, "[CombinedRobotDescription]: Map##_helper error!");    \
  };                                                                                                       \
  template <typename Key_, typename Value_>                                                                \
  struct Map##_helper<Key_, Value_, std::true_type, std::true_type> {                                      \
    using type = std_utils::ctm_combine_t<                                                                 \
        Key_, Value_, typename ConcreteTopology1::Map, typename ConcreteTopology2::Map,                    \
        mapSub##Key##To##Key##_t<ConcreteTopology1>, mapSub##Key##To##Key##_t<ConcreteTopology2>,          \
        mapSub##Value##To##Value##_t<ConcreteTopology1>, mapSub##Value##To##Value##_t<ConcreteTopology2>>; \
  };                                                                                                       \
  template <typename Key_, typename Value_>                                                                \
  struct Map##_helper<Key_, Value_, std::true_type, std::false_type> {                                     \
    using type = std_utils::ctm_transform_t<Key_, Value_, typename ConcreteTopology1::Map,                 \
                                            mapSub##Key##To##Key##_t<ConcreteTopology1>,                   \
                                            mapSub##Value##To##Value##_t<ConcreteTopology1>>;              \
  };                                                                                                       \
  template <typename Key_, typename Value_>                                                                \
  struct Map##_helper<Key_, Value_, std::false_type, std::true_type> {                                     \
    using type = std_utils::ctm_transform_t<Key_, Value_, typename ConcreteTopology2::Map,                 \
                                            mapSub##Key##To##Key##_t<ConcreteTopology2>,                   \
                                            mapSub##Value##To##Value##_t<ConcreteTopology2>>;              \
  };                                                                                                       \
  template <typename Key_, typename Value_>                                                                \
  struct Map##_helper<Key_, Value_, std::false_type, std::false_type> {                                    \
    using type = void;                                                                                     \
  };                                                                                                       \
  STD_UTILS_HAS_TYPE(Map);                                                                                 \
 public:                                                                                                   \
  using Map = typename Map##_helper<Key, Value,                                                            \
    has_##Map##_t<ConcreteTopology1>, has_##Map##_t<ConcreteTopology2>>::type;

#define ROMO_CREATE_KEY_COMBINED_MAPS_CONDITIONALLY(Key, Value, Map, Summand)                                       \
 private:                                                                                                           \
  template <typename Key_, typename Value_, typename hasType1, typename hasType2>                                   \
  struct Map##_helper {                                                                                             \
    using type = std_utils::CompileTimeMap<Key_, Value_>;                                                           \
    static_assert(!std::is_same<Key_, Key_>::value, "[CombinedRobotDescription]: ##Map##_helper error!");           \
  };                                                                                                                \
  template <typename Key_, typename Value_>                                                                         \
  struct Map##_helper<Key_, Value_, std::true_type, std::true_type> {                                               \
    using type = std_utils::ctm_combine_t<Key_, Value_, typename ConcreteTopology1::Map,                            \
                                          std_utils::ctm_add_to_values_t<typename ConcreteTopology2::Key, Value,    \
                                                                         typename ConcreteTopology2::Map, Summand>, \
                                          mapSub##Key##To##Key##_t<ConcreteTopology1>,                              \
                                          mapSub##Key##To##Key##_t<ConcreteTopology2>>;                             \
  };                                                                                                                \
  template <typename Key_, typename Value_>                                                                         \
  struct Map##_helper<Key_, Value_, std::true_type, std::false_type> {                                              \
    using type = std_utils::ctm_transform_t<Key_, Value_, typename ConcreteTopology1::Map,                          \
                                            mapSub##Key##To##Key##_t<ConcreteTopology1>>;                           \
  };                                                                                                                \
  template <typename Key_, typename Value_>                                                                         \
  struct Map##_helper<Key_, Value_, std::false_type, std::true_type> {                                              \
    using type = std_utils::ctm_transform_t<Key_, Value_, typename ConcreteTopology2::Map,                          \
                                            mapSub##Key##To##Key##_t<ConcreteTopology2>>;                           \
  };                                                                                                                \
  template <typename Key_, typename Value_>                                                                         \
  struct Map##_helper<Key_, Value_, std::false_type, std::false_type> {                                             \
    using type = void;                                                                                              \
  };                                                                                                                \
  STD_UTILS_HAS_TYPE(Map);                                                                                          \
 public:                                                                                                            \
  using Map = typename Map##_helper<Key, Value,                                                                     \
        has_##Map##_t<ConcreteTopology1>, has_##Map##_t<ConcreteTopology2>>::type;

namespace romo {

template <typename LimbEnum_, typename BranchEnum_, typename JointNodeEnum_, typename JointEnum_,
          typename ActuatorNodeEnum_, typename ActuatorEnum_, typename BodyNodeEnum_, typename BodyEnum_,
          typename GeneralizedCoordinatesEnum_, typename GeneralizedVelocitiesEnum_, typename ContactEnum_>
struct CombinedEnums {
  using LimbEnum = LimbEnum_;
  using BranchEnum = BranchEnum_;
  using JointNodeEnum = JointNodeEnum_;
  using JointEnum = JointEnum_;
  using ActuatorNodeEnum = ActuatorNodeEnum_;
  using ActuatorEnum = ActuatorEnum_;
  using BodyNodeEnum = BodyNodeEnum_;
  using BodyEnum = BodyEnum_;
  using GeneralizedCoordinatesEnum = GeneralizedCoordinatesEnum_;
  using GeneralizedVelocitiesEnum = GeneralizedVelocitiesEnum_;
  using ContactEnum = ContactEnum_;
};

template <typename CombinedEnums_, typename ConcreteTopology1_, typename ConcreteTopology2_>
struct CombinedTopology {
  //! Expose Topology Types
  using ConcreteTopology1 = ConcreteTopology1_;
  using ConcreteTopology2 = ConcreteTopology2_;

  //! Expose Enum Types
  using LimbEnum = typename CombinedEnums_::LimbEnum;
  using BranchEnum = typename CombinedEnums_::BranchEnum;
  using JointNodeEnum = typename CombinedEnums_::JointNodeEnum;
  using JointEnum = typename CombinedEnums_::JointEnum;
  using ActuatorNodeEnum = typename CombinedEnums_::ActuatorNodeEnum;
  using ActuatorEnum = typename CombinedEnums_::ActuatorEnum;
  using BodyNodeEnum = typename CombinedEnums_::BodyNodeEnum;
  using BodyEnum = typename CombinedEnums_::BodyEnum;
  using GeneralizedCoordinatesEnum = typename CombinedEnums_::GeneralizedCoordinatesEnum;
  using GeneralizedVelocitiesEnum = typename CombinedEnums_::GeneralizedVelocitiesEnum;
  using ContactEnum = typename CombinedEnums_::ContactEnum;
  using ContactStateEnum = typename ConcreteTopology1::ContactStateEnum;
  using CoordinateFrameEnum = typename ConcreteTopology1::CoordinateFrameEnum;

  ROMO_CREATE_COMBINED_KEYS(LimbEnum, limbKeys);
  ROMO_CREATE_COMBINED_KEYS_SKIP_BASE(BranchEnum, branchKeys);
  ROMO_CREATE_COMBINED_KEYS(JointNodeEnum, jointNodeKeys);
  ROMO_CREATE_COMBINED_KEYS(JointEnum, jointKeys);
  ROMO_CREATE_COMBINED_KEYS(ActuatorNodeEnum, actuatorNodeKeys);
  ROMO_CREATE_COMBINED_KEYS(ActuatorEnum, actuatorKeys);
  ROMO_CREATE_COMBINED_KEYS_SKIP_BASE(BodyNodeEnum, bodyNodeKeys);
  ROMO_CREATE_COMBINED_KEYS_SKIP_BASE(BodyEnum, bodyKeys);
  ROMO_CREATE_COMBINED_KEYS(GeneralizedCoordinatesEnum, generalizedCoordinateKeys);
  ROMO_CREATE_COMBINED_KEYS(GeneralizedVelocitiesEnum, generalizedVelocityKeys);
  ROMO_CREATE_COMBINED_KEYS(ContactEnum, contactKeys);

  // Those are not comined but forwarded from the first definition
  // #TODO build proper super set
 private:
  using contactStateKeys_type = decltype(ConcreteTopology1::contactStateKeys);
  using coordinateFrameKeys_type = decltype(ConcreteTopology1::coordinateFrameKeys);
 public:
  static constexpr contactStateKeys_type contactStateKeys = ConcreteTopology1::contactStateKeys;
  static constexpr coordinateFrameKeys_type coordinateFrameKeys = ConcreteTopology1::coordinateFrameKeys;

  // Combine maps
  ROMO_CREATE_COMBINED_MAPS_SKIP_BASE(BranchEnum, LimbEnum, mapBranchEnumToLimbEnum);
  ROMO_CREATE_COMBINED_MAPS(JointEnum, ActuatorEnum, mapJointEnumToActuatorEnum);
  ROMO_CREATE_COMBINED_MAPS(ActuatorEnum, LimbEnum, mapActuatorEnumToLimbEnum);
  ROMO_CREATE_COMBINED_MAPS(ActuatorEnum, ActuatorNodeEnum, mapActuatorEnumToActuatorNodeEnum);
  ROMO_CREATE_COMBINED_MAPS(JointEnum, JointNodeEnum, mapJointEnumToJointNodeEnum);
  ROMO_CREATE_COMBINED_MAPS_SKIP_BASE(BodyEnum, BranchEnum, mapBodyEnumToBranchEnum);
  ROMO_CREATE_COMBINED_MAPS_SKIP_BASE(BodyEnum, BodyNodeEnum, mapBodyEnumToBodyNodeEnum);
  ROMO_CREATE_COMBINED_MAPS(ContactEnum, BodyEnum, mapContactEnumToBodyEnum);
  ROMO_CREATE_COMBINED_MAPS_CONDITIONALLY(LimbEnum, JointEnum, mapLimbToStartJoint);

  ROMO_CREATE_KEY_COMBINED_MAPS_CONDITIONALLY(LimbEnum, unsigned int, mapLimbToNDof, 0u);

  ROMO_CREATE_KEY_COMBINED_MAPS_CONDITIONALLY(LimbEnum, unsigned int, mapLimbToStartIndexInJ,
                                              static_cast<unsigned int>(ConcreteTopology1::JointEnum::SIZE));
  ROMO_CREATE_KEY_COMBINED_MAPS_CONDITIONALLY(LimbEnum, unsigned int, mapLimbToWheelIndexInJ,
                                              static_cast<unsigned int>(ConcreteTopology1::JointEnum::SIZE));
  ROMO_CREATE_KEY_COMBINED_MAPS_CONDITIONALLY(
      BranchEnum, unsigned int, mapBranchToStartIndexInU,
      static_cast<unsigned int>(ConcreteTopology1::GeneralizedVelocitiesEnum::SIZE));

  // Create start and end body maps
  ROMO_CREATE_COMBINED_MAPS_SKIP_BASE(BranchEnum, BodyEnum, mapBranchToStartBody);
  ROMO_CREATE_COMBINED_MAPS_SKIP_BASE(BranchEnum, BodyEnum, mapBranchToEndBody);
};

template <typename CombinedEnums_, typename ConcreteTopology1_, typename ConcreteTopology2_>
constexpr typename CombinedTopology<CombinedEnums_, ConcreteTopology1_, ConcreteTopology2_>::limbKeys_type
    CombinedTopology<CombinedEnums_, ConcreteTopology1_, ConcreteTopology2_>::limbKeys;

template <typename CombinedEnums_, typename ConcreteTopology1_, typename ConcreteTopology2_>
constexpr typename CombinedTopology<CombinedEnums_, ConcreteTopology1_, ConcreteTopology2_>::branchKeys_type
    CombinedTopology<CombinedEnums_, ConcreteTopology1_, ConcreteTopology2_>::branchKeys;

template <typename CombinedEnums_, typename ConcreteTopology1_, typename ConcreteTopology2_>
constexpr typename CombinedTopology<CombinedEnums_, ConcreteTopology1_, ConcreteTopology2_>::jointNodeKeys_type
    CombinedTopology<CombinedEnums_, ConcreteTopology1_, ConcreteTopology2_>::jointNodeKeys;

template <typename CombinedEnums_, typename ConcreteTopology1_, typename ConcreteTopology2_>
constexpr typename CombinedTopology<CombinedEnums_, ConcreteTopology1_, ConcreteTopology2_>::jointKeys_type
    CombinedTopology<CombinedEnums_, ConcreteTopology1_, ConcreteTopology2_>::jointKeys;

template <typename CombinedEnums_, typename ConcreteTopology1_, typename ConcreteTopology2_>
constexpr typename CombinedTopology<CombinedEnums_, ConcreteTopology1_, ConcreteTopology2_>::actuatorNodeKeys_type
    CombinedTopology<CombinedEnums_, ConcreteTopology1_, ConcreteTopology2_>::actuatorNodeKeys;

template <typename CombinedEnums_, typename ConcreteTopology1_, typename ConcreteTopology2_>
constexpr typename CombinedTopology<CombinedEnums_, ConcreteTopology1_, ConcreteTopology2_>::actuatorKeys_type
    CombinedTopology<CombinedEnums_, ConcreteTopology1_, ConcreteTopology2_>::actuatorKeys;

template <typename CombinedEnums_, typename ConcreteTopology1_, typename ConcreteTopology2_>
constexpr typename CombinedTopology<CombinedEnums_, ConcreteTopology1_, ConcreteTopology2_>::bodyNodeKeys_type
    CombinedTopology<CombinedEnums_, ConcreteTopology1_, ConcreteTopology2_>::bodyNodeKeys;

template <typename CombinedEnums_, typename ConcreteTopology1_, typename ConcreteTopology2_>
constexpr typename CombinedTopology<CombinedEnums_, ConcreteTopology1_, ConcreteTopology2_>::bodyKeys_type
    CombinedTopology<CombinedEnums_, ConcreteTopology1_, ConcreteTopology2_>::bodyKeys;

template <typename CombinedEnums_, typename ConcreteTopology1_, typename ConcreteTopology2_>
constexpr typename CombinedTopology<CombinedEnums_, ConcreteTopology1_, ConcreteTopology2_>::generalizedCoordinateKeys_type
    CombinedTopology<CombinedEnums_, ConcreteTopology1_, ConcreteTopology2_>::generalizedCoordinateKeys;

template <typename CombinedEnums_, typename ConcreteTopology1_, typename ConcreteTopology2_>
constexpr typename CombinedTopology<CombinedEnums_, ConcreteTopology1_, ConcreteTopology2_>::generalizedVelocityKeys_type
    CombinedTopology<CombinedEnums_, ConcreteTopology1_, ConcreteTopology2_>::generalizedVelocityKeys;

template <typename CombinedEnums_, typename ConcreteTopology1_, typename ConcreteTopology2_>
constexpr typename CombinedTopology<CombinedEnums_, ConcreteTopology1_, ConcreteTopology2_>::contactKeys_type
    CombinedTopology<CombinedEnums_, ConcreteTopology1_, ConcreteTopology2_>::contactKeys;

template <typename CombinedEnums_, typename ConcreteTopology1_, typename ConcreteTopology2_>
constexpr typename CombinedTopology<CombinedEnums_, ConcreteTopology1_, ConcreteTopology2_>::contactStateKeys_type
    CombinedTopology<CombinedEnums_, ConcreteTopology1_, ConcreteTopology2_>::contactStateKeys;

template <typename CombinedEnums_, typename ConcreteTopology1_, typename ConcreteTopology2_>
constexpr typename CombinedTopology<CombinedEnums_, ConcreteTopology1_, ConcreteTopology2_>::coordinateFrameKeys_type
    CombinedTopology<CombinedEnums_, ConcreteTopology1_, ConcreteTopology2_>::coordinateFrameKeys;

template <typename CombinedTopology_, typename ConcreteDefinitions1_, typename ConcreteDefinitions2_>
struct CombinedDefinitions {
 protected:
  using CombinedTopology = CombinedTopology_;
  using BranchEnum = typename CombinedTopology::BranchEnum;
  using LimbEnum = typename CombinedTopology::LimbEnum;
  using BodyEnum = typename CombinedTopology::BodyEnum;
  using JointEnum = typename CombinedTopology::JointEnum;
  using ConcreteDefinitions1 = ConcreteDefinitions1_;
  using ConcreteDefinitions2 = ConcreteDefinitions2_;

 public:
  inline static constexpr unsigned int getNumLegsImpl() {
    return ConcreteDefinitions1::getNumLegsImpl() + ConcreteDefinitions2::getNumLegsImpl();
  }
  inline static constexpr unsigned int getNumArmsImpl() {
    return ConcreteDefinitions1::getNumArmsImpl() + ConcreteDefinitions2::getNumArmsImpl();
  }
  inline static constexpr unsigned int getNumDofImpl() {
    return ConcreteDefinitions1::getNumDofImpl() + ConcreteDefinitions2::getNumDofImpl();
  }

  inline static constexpr unsigned int getBaseGeneralizedCoordinatesDimensionImpl() {
    return ConcreteDefinitions1::getBaseGeneralizedCoordinatesDimensionImpl() +
           ConcreteDefinitions2::getBaseGeneralizedCoordinatesDimensionImpl();
  }

  inline static constexpr unsigned int getBaseGeneralizedVelocitiesDimensionImpl() {
    return ConcreteDefinitions1::getBaseGeneralizedVelocitiesDimensionImpl() +
           ConcreteDefinitions2::getBaseGeneralizedVelocitiesDimensionImpl();
  }

  inline static constexpr unsigned int getNumDofLimbImpl(LimbEnum limb) {
    return CombinedTopology::mapLimbToNDof::at(limb);
  }
  inline static constexpr unsigned int getBranchStartIndexInUImpl(BranchEnum branch) {
    return CombinedTopology::mapBranchToStartIndexInU::at(branch);
  }
  inline static constexpr unsigned int getLimbStartIndexInJImpl(LimbEnum limb) {
    return CombinedTopology::mapLimbToStartIndexInJ::at(limb);
  }
  inline static constexpr unsigned int getWheelIndexInJImpl(LimbEnum limb) {
    return CombinedTopology::mapLimbToWheelIndexInJ::at(limb);
  }
  inline static constexpr BodyEnum getBranchStartBodyImpl(BranchEnum branch) {
    return CombinedTopology::mapBranchToStartBody::at(branch);
  }
  inline static constexpr BodyEnum getBranchEndBodyImpl(BranchEnum branch) {
    return CombinedTopology::mapBranchToEndBody::at(branch);
  }
  inline static constexpr JointEnum getLimbStartJointImpl(LimbEnum limb) {
    return CombinedTopology::mapLimbToStartJoint::at(limb);
  }

  // M545 specifics
  inline static constexpr double getWheelDiameterImpl() { return ConcreteDefinitions1::getWheelDiameterImpl(); }
};

}  // namespace romo
