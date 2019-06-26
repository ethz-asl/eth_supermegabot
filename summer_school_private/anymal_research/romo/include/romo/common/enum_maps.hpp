namespace romo {

namespace internal {

/*** This file contains all maps that are available:
 *  It forces the user to implement the following mappings as a std_utils::CompileTimeMaps
 *      -> mapBranchEnumToLimbEnum
 *      -> mapJointEnumToActuatorEnum
 *      -> mapActuatorEnumToLimbEnum
 *      -> mapJointEnumToJointNodeEnum
 *      -> mapActuatorEnumToActuatorNodeEnum
 *      -> mapBodyEnumToBranchEnum
 *      -> mapBodyEnumToBodyNodeEnum
 *      -> mapContactEnumToBodyEnum
 */

/*  Map Enum helper
 *  Static assert if the requested map is not implemented
 */
template< typename fromEnum_, typename toEnum_, typename ConcreteTopology_ >
struct MapEnumHelper {
  static_assert(sizeof(fromEnum_) != sizeof(fromEnum_), "Mapping of the enums is not defined!");
};

/*  Map Enum helper specialization ( BranchEnum <-> LimbEnum )
 *  Mapping must be unique / bidirectional!
 *  REQUIRES ConcreteTopology to implement Compile Time Map: mapBranchEnumToLimbEnum
 */
template< typename ConcreteTopology_ >
struct MapEnumHelper<typename ConcreteTopology_::BranchEnum,
                     typename ConcreteTopology_::LimbEnum,
                     ConcreteTopology_>
{
  static_assert(ConcreteTopology_::mapBranchEnumToLimbEnum::allValuesUnique(),
                "Mapping BranchEnum <-> LimbEnum must be unique / bidirectional!");

  static constexpr typename ConcreteTopology_::LimbEnum map( typename ConcreteTopology_::BranchEnum branchEnum ) {
    return ConcreteTopology_::mapBranchEnumToLimbEnum::at(branchEnum);
  }
};

template< typename ConcreteTopology_ >
struct MapEnumHelper<typename ConcreteTopology_::LimbEnum,
                     typename ConcreteTopology_::BranchEnum,
                     ConcreteTopology_>
{
  static_assert(ConcreteTopology_::mapBranchEnumToLimbEnum::allValuesUnique(),
                "Mapping BranchEnum <-> LimbEnum must be unique / bidirectional!");

  static constexpr typename ConcreteTopology_::BranchEnum map( typename ConcreteTopology_::LimbEnum limbEnum ) {
    return ConcreteTopology_::mapBranchEnumToLimbEnum::find(limbEnum);
  }
};

/*  Map Enum helper specialization ( JointEnum <-> ActuatorEnum )
 *  Mapping must be unique / bidirectional!
 *  REQUIRES ConcreteTopology to implement Compile Time Map: mapJointEnumToActuatorEnum
 */
template< typename ConcreteTopology_ >
struct MapEnumHelper<typename ConcreteTopology_::JointEnum,
                     typename ConcreteTopology_::ActuatorEnum,
                     ConcreteTopology_>
{
  static_assert(ConcreteTopology_::mapJointEnumToActuatorEnum::allValuesUnique(),
                "Mapping JointEnum <-> ActuatorEnum must be unique / bidirectional!");

  static constexpr typename ConcreteTopology_::ActuatorEnum map( typename ConcreteTopology_::JointEnum jointEnum ) {
    return ConcreteTopology_::mapJointEnumToActuatorEnum::at(jointEnum);
  }
};

template< typename ConcreteTopology_ >
struct MapEnumHelper<typename ConcreteTopology_::ActuatorEnum,
                     typename ConcreteTopology_::JointEnum,
                     ConcreteTopology_>
{
  static_assert(ConcreteTopology_::mapJointEnumToActuatorEnum::allValuesUnique(),
                "Mapping JointEnum <-> ActuatorEnum must be unique / bidirectional!");

  static constexpr typename ConcreteTopology_::JointEnum map( typename ConcreteTopology_::ActuatorEnum actuatorEnum ) {
    return ConcreteTopology_::mapJointEnumToActuatorEnum::find(actuatorEnum);
  }
};

/*  Map Enum helper specialization ( ActuatorEnum -> LimbEnum )
 *  REQUIRES ConcreteTopology to implement Compile Time Map: mapActuatorEnumToLimbEnum
 */
template< typename ConcreteTopology_ >
struct MapEnumHelper<typename ConcreteTopology_::ActuatorEnum,
                     typename ConcreteTopology_::LimbEnum,
                     ConcreteTopology_>
{
  static constexpr typename ConcreteTopology_::LimbEnum map( typename ConcreteTopology_::ActuatorEnum actuatorEnum ) {
    return ConcreteTopology_::mapActuatorEnumToLimbEnum::at(actuatorEnum);
  }
};

/*  Map Enum helper specialization ( ActuatorEnum -> ActuatorNodeEnum )
 *  REQUIRES ConcreteTopology to implement Compile Time Map: mapActuatorEnumToActuatorNodeEnum
 */
template< typename ConcreteTopology_ >
struct MapEnumHelper<typename ConcreteTopology_::ActuatorEnum,
                     typename ConcreteTopology_::ActuatorNodeEnum,
                     ConcreteTopology_>
{
  static constexpr typename ConcreteTopology_::ActuatorNodeEnum map( typename ConcreteTopology_::ActuatorEnum actuatorEnum ) {
    return ConcreteTopology_::mapActuatorEnumToActuatorNodeEnum::at(actuatorEnum);
  }
};

/*  Map Enum helper specialization ( JointEnum -> JointNodeEnum )
 *  REQUIRES ConcreteTopology to implement Compile Time Map: mapJointEnumToJointNodeEnum
 */
template< typename ConcreteTopology_ >
struct MapEnumHelper<typename ConcreteTopology_::JointEnum,
                     typename ConcreteTopology_::JointNodeEnum,
                     ConcreteTopology_>
{
  static constexpr typename ConcreteTopology_::JointNodeEnum map( typename ConcreteTopology_::JointEnum jointEnum ) {
    return ConcreteTopology_::mapJointEnumToJointNodeEnum::at(jointEnum);
  }
};

/*  Map Enum helper specialization ( BodyEnum -> BranchEnum )
 *  REQUIRES ConcreteTopology to implement Compile Time Map: mapBodyEnumToBranchEnum
 */
template< typename ConcreteTopology_ >
struct MapEnumHelper<typename ConcreteTopology_::BodyEnum,
                     typename ConcreteTopology_::BranchEnum,
                     ConcreteTopology_>
{
  static constexpr typename ConcreteTopology_::BranchEnum map( typename ConcreteTopology_::BodyEnum bodyEnum ) {
    return ConcreteTopology_::mapBodyEnumToBranchEnum::at(bodyEnum);
  }
};

/*  Map Enum helper specialization ( BodyEnum -> BodyNodeEnum )
 *  REQUIRES ConcreteTopology to implement Compile Time Map: mapBodyEnumToBodyNodeEnum
 */
template< typename ConcreteTopology_ >
struct MapEnumHelper<typename ConcreteTopology_::BodyEnum,
                     typename ConcreteTopology_::BodyNodeEnum,
                     ConcreteTopology_>
{
  static constexpr typename ConcreteTopology_::BodyNodeEnum map( typename ConcreteTopology_::BodyEnum bodyEnum ) {
    return ConcreteTopology_::mapBodyEnumToBodyNodeEnum::at(bodyEnum);
  }
};

/*  Map Enum helper specialization ( ContactEnum -> BodyEnum )
 *  REQUIRES ConcreteTopology to implement Compile Time Map: mapContactEnumToBodyEnum
 */
template< typename ConcreteTopology_ >
struct MapEnumHelper<typename ConcreteTopology_::ContactEnum,
                     typename ConcreteTopology_::BodyEnum,
                     ConcreteTopology_>
{
  static constexpr typename ConcreteTopology_::BodyEnum map( typename ConcreteTopology_::ContactEnum contactEnum ) {
    return ConcreteTopology_::mapContactEnumToBodyEnum::at(contactEnum);
  }
};

/* By combining these maps even more mappings can be achieved!
 */

/*  Map Enum helper specialization ( ContactEnum -> BodyNodeEnum )
 */
template< typename ConcreteTopology_ >
struct MapEnumHelper<typename ConcreteTopology_::ContactEnum,
                     typename ConcreteTopology_::BodyNodeEnum,
                     ConcreteTopology_>
{
  static constexpr typename ConcreteTopology_::BodyNodeEnum map( typename ConcreteTopology_::ContactEnum contactEnum ) {
    return MapEnumHelper<typename ConcreteTopology_::BodyEnum, typename ConcreteTopology_::BodyNodeEnum, ConcreteTopology_>::map(
      MapEnumHelper<typename ConcreteTopology_::ContactEnum, typename ConcreteTopology_::BodyEnum, ConcreteTopology_>::map(contactEnum));
  }
};

/*  Map Enum helper specialization ( ContactEnum -> BranchEnum )
 */
template< typename ConcreteTopology_ >
struct MapEnumHelper<typename ConcreteTopology_::ContactEnum,
                     typename ConcreteTopology_::BranchEnum,
                     ConcreteTopology_>
{
  static constexpr typename ConcreteTopology_::BranchEnum map( typename ConcreteTopology_::ContactEnum contactEnum ) {
    return MapEnumHelper<typename ConcreteTopology_::BodyEnum, typename ConcreteTopology_::BranchEnum, ConcreteTopology_>::map(
      MapEnumHelper<typename ConcreteTopology_::ContactEnum, typename ConcreteTopology_::BodyEnum, ConcreteTopology_>::map(contactEnum));
  }
};

/*  Map Enum helper specialization ( ContactEnum -> LimbEnum )
 */
template< typename ConcreteTopology_ >
struct MapEnumHelper<typename ConcreteTopology_::ContactEnum,
                     typename ConcreteTopology_::LimbEnum,
                     ConcreteTopology_>
{
  static constexpr typename ConcreteTopology_::LimbEnum map( typename ConcreteTopology_::ContactEnum contactEnum ) {
    return MapEnumHelper<typename ConcreteTopology_::BranchEnum, typename ConcreteTopology_::LimbEnum, ConcreteTopology_>::map(
      MapEnumHelper<typename ConcreteTopology_::ContactEnum, typename ConcreteTopology_::BranchEnum, ConcreteTopology_>::map(contactEnum));
  }
};

/*  Map Enum helper specialization ( BodyEnum -> LimbEnum )
 */
template< typename ConcreteTopology_ >
struct MapEnumHelper<typename ConcreteTopology_::BodyEnum,
                     typename ConcreteTopology_::LimbEnum,
                     ConcreteTopology_>
{
  static constexpr typename ConcreteTopology_::LimbEnum map( typename ConcreteTopology_::BodyEnum bodyEnum ) {
    return MapEnumHelper<typename ConcreteTopology_::BranchEnum, typename ConcreteTopology_::LimbEnum, ConcreteTopology_>::map(
      MapEnumHelper<typename ConcreteTopology_::BodyEnum, typename ConcreteTopology_::BranchEnum, ConcreteTopology_>::map(bodyEnum));
  }
};

/*  Map Enum helper specialization ( ActuatorEnum -> BranchEnum )
 */
template< typename ConcreteTopology_ >
struct MapEnumHelper<typename ConcreteTopology_::ActuatorEnum,
                     typename ConcreteTopology_::BranchEnum,
                     ConcreteTopology_>
{
  static constexpr typename ConcreteTopology_::BranchEnum map( typename ConcreteTopology_::ActuatorEnum actuatorEnum ) {
    return MapEnumHelper<typename ConcreteTopology_::LimbEnum, typename ConcreteTopology_::BranchEnum, ConcreteTopology_>::map(
      MapEnumHelper<typename ConcreteTopology_::ActuatorEnum, typename ConcreteTopology_::LimbEnum, ConcreteTopology_>::map(actuatorEnum));
  }
};

/*  Map Enum helper specialization ( ActuatorEnum -> JointNodeEnum )
 */
template< typename ConcreteTopology_ >
struct MapEnumHelper<typename ConcreteTopology_::ActuatorEnum,
                     typename ConcreteTopology_::JointNodeEnum,
                     ConcreteTopology_>
{
  static constexpr typename ConcreteTopology_::JointNodeEnum map( typename ConcreteTopology_::ActuatorEnum actuatorEnum ) {
    return MapEnumHelper<typename ConcreteTopology_::JointEnum, typename ConcreteTopology_::JointNodeEnum, ConcreteTopology_>::map(
      MapEnumHelper<typename ConcreteTopology_::ActuatorEnum, typename ConcreteTopology_::JointEnum, ConcreteTopology_>::map(actuatorEnum));
  }
};

/*  Map Enum helper specialization ( JointEnum -> ActuatorNodeEnum )
 */
template< typename ConcreteTopology_ >
struct MapEnumHelper<typename ConcreteTopology_::JointEnum,
                     typename ConcreteTopology_::ActuatorNodeEnum,
                     ConcreteTopology_>
{
  static constexpr typename ConcreteTopology_::ActuatorNodeEnum map( typename ConcreteTopology_::JointEnum jointEnum ) {
    return MapEnumHelper<typename ConcreteTopology_::ActuatorEnum, typename ConcreteTopology_::ActuatorNodeEnum, ConcreteTopology_>::map(
        MapEnumHelper<typename ConcreteTopology_::JointEnum, typename ConcreteTopology_::ActuatorEnum, ConcreteTopology_>::map(jointEnum));
  }
};

/*  Map Enum helper specialization ( JointEnum -> LimbEnum )
 */
template< typename ConcreteTopology_ >
struct MapEnumHelper<typename ConcreteTopology_::JointEnum,
                     typename ConcreteTopology_::LimbEnum,
                     ConcreteTopology_>
{
  static constexpr typename ConcreteTopology_::LimbEnum map( typename ConcreteTopology_::JointEnum jointEnum ) {
    return MapEnumHelper<typename ConcreteTopology_::ActuatorEnum, typename ConcreteTopology_::LimbEnum, ConcreteTopology_>::map(
      MapEnumHelper<typename ConcreteTopology_::JointEnum, typename ConcreteTopology_::ActuatorEnum, ConcreteTopology_>::map(jointEnum));
  }
};

/*  Map Enum helper specialization ( JointEnum -> BranchEnum )
 */
template< typename ConcreteTopology_ >
struct MapEnumHelper<typename ConcreteTopology_::JointEnum,
                     typename ConcreteTopology_::BranchEnum,
                     ConcreteTopology_>
{
  static constexpr typename ConcreteTopology_::BranchEnum map( typename ConcreteTopology_::JointEnum jointEnum ) {
    return MapEnumHelper<typename ConcreteTopology_::LimbEnum, typename ConcreteTopology_::BranchEnum, ConcreteTopology_>::map(
      MapEnumHelper<typename ConcreteTopology_::JointEnum, typename ConcreteTopology_::LimbEnum, ConcreteTopology_>::map(jointEnum));
  }
};

}

}
