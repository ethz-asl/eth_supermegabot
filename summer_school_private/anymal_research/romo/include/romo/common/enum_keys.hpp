#include "std_utils/std_utils.hpp"

namespace romo {

namespace internal {

/*** This file contains all key arrays that are available:
 *  It forces the user to implement the following keyArrays as a std_utils::KeyArray
*       -> limbKeys
*       -> branchKeys
*       -> jointNodeKeys
*       -> jointKeys
*       -> actuatorNodeKeys
*       -> actuatorKeys
*       -> bodyNodeKeys
*       -> bodyKeys
*       -> generalizedCoordinateKeys
*       -> generalizedVelocityKeys
*       -> contactKeys
*       -> contactStateKeys
*       -> coordinateFrameKeys
*/

// Key helper
template <typename ConcreteTopology_, typename Enum_> struct KeysHelper {
  inline static constexpr std_utils::KeyArray<Enum_> keys() {
    static_assert( sizeof(Enum_) != sizeof(Enum_) , "Keys are not defined for this enum!");
    return std_utils::KeyArray<Enum_>({{}});
  }
};
template <typename ConcreteTopology_> struct KeysHelper<ConcreteTopology_, typename ConcreteTopology_::LimbEnum> {
  inline static constexpr std_utils::KeyArray<typename ConcreteTopology_::LimbEnum> keys() { return ConcreteTopology_::limbKeys; }
};
template <typename ConcreteTopology_> struct KeysHelper<ConcreteTopology_, typename ConcreteTopology_::BranchEnum> {
  inline static constexpr std_utils::KeyArray<typename ConcreteTopology_::BranchEnum> keys() { return ConcreteTopology_::branchKeys; }
};
template <typename ConcreteTopology_> struct KeysHelper<ConcreteTopology_, typename ConcreteTopology_::JointNodeEnum> {
  inline static constexpr std_utils::KeyArray<typename ConcreteTopology_::JointNodeEnum> keys() { return ConcreteTopology_::jointNodeKeys; }
};
template <typename ConcreteTopology_> struct KeysHelper<ConcreteTopology_, typename ConcreteTopology_::JointEnum> {
  inline static constexpr std_utils::KeyArray<typename ConcreteTopology_::JointEnum> keys() { return ConcreteTopology_::jointKeys; }
};
template <typename ConcreteTopology_> struct KeysHelper<ConcreteTopology_, typename ConcreteTopology_::ActuatorNodeEnum> {
  inline static constexpr std_utils::KeyArray<typename ConcreteTopology_::ActuatorNodeEnum> keys() { return ConcreteTopology_::actuatorNodeKeys; }
};
template <typename ConcreteTopology_> struct KeysHelper<ConcreteTopology_, typename ConcreteTopology_::ActuatorEnum> {
  inline static constexpr std_utils::KeyArray<typename ConcreteTopology_::ActuatorEnum> keys() { return ConcreteTopology_::actuatorKeys; }
};
template <typename ConcreteTopology_> struct KeysHelper<ConcreteTopology_, typename ConcreteTopology_::BodyNodeEnum> {
  inline static constexpr std_utils::KeyArray<typename ConcreteTopology_::BodyNodeEnum> keys() { return ConcreteTopology_::bodyNodeKeys; }
};
template <typename ConcreteTopology_> struct KeysHelper<ConcreteTopology_, typename ConcreteTopology_::BodyEnum> {
  inline static constexpr std_utils::KeyArray<typename ConcreteTopology_::BodyEnum> keys() { return ConcreteTopology_::bodyKeys; }
};
template <typename ConcreteTopology_> struct KeysHelper<ConcreteTopology_, typename ConcreteTopology_::GeneralizedCoordinatesEnum> {
  inline static constexpr std_utils::KeyArray<typename ConcreteTopology_::GeneralizedCoordinatesEnum> keys() { return ConcreteTopology_::generalizedCoordinateKeys; }
};
template <typename ConcreteTopology_> struct KeysHelper<ConcreteTopology_, typename ConcreteTopology_::GeneralizedVelocitiesEnum> {
  inline static constexpr std_utils::KeyArray<typename ConcreteTopology_::GeneralizedVelocitiesEnum> keys() { return ConcreteTopology_::generalizedVelocityKeys; }
};
template <typename ConcreteTopology_> struct KeysHelper<ConcreteTopology_, typename ConcreteTopology_::ContactEnum> {
  inline static constexpr std_utils::KeyArray<typename ConcreteTopology_::ContactEnum> keys() { return ConcreteTopology_::contactKeys; }
};
template <typename ConcreteTopology_> struct KeysHelper<ConcreteTopology_, typename ConcreteTopology_::ContactStateEnum> {
  inline static constexpr std_utils::KeyArray<typename ConcreteTopology_::ContactStateEnum> keys() { return ConcreteTopology_::contactStateKeys; }
};
template <typename ConcreteTopology_> struct KeysHelper<ConcreteTopology_, typename ConcreteTopology_::CoordinateFrameEnum> {
  inline static constexpr std_utils::KeyArray<typename ConcreteTopology_::CoordinateFrameEnum> keys() { return ConcreteTopology_::coordinateFrameKeys; }
};

}  // namespace internal

} // namespace romo