/*!
* @file     DerivativeTraits.hpp
* @author   Perry Franklin
* @date     Feb 7, 2019
* @brief
*/
#pragma once

#include <kindr/Core>

namespace robot_utils {

template <typename ValueType>
struct DerivativeTraits{
  using type = ValueType;
};

template < typename PrimType_ , int Dimension_>
struct DerivativeTraits< kindr::Position< PrimType_, Dimension_> >{
  using type = kindr::Velocity<PrimType_, Dimension_>;
};

template < typename PrimType_  >
struct DerivativeTraits< kindr::RotationQuaternion< PrimType_> >{
  using type = kindr::LocalAngularVelocity< PrimType_ >;
};

template < typename PrimType_, typename Position_, typename Rotation_ >
struct DerivativeTraits< kindr::HomogeneousTransformation<PrimType_, Position_, Rotation_ > >{
  using type = kindr::TwistLinearVelocityLocalAngularVelocity< PrimType_ >;
};

}  // robot_utils
