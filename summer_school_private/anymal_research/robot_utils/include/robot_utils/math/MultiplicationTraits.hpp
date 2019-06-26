/*
 * MultiplicationTraits.hpp
 *
 *  Created on: Feb 12, 2019
 *      Author: Perry Franklin
 */

#pragma once

#include <type_traits>

#include <kindr/Core>
#include <robot_utils/math/DerivativeTraits.hpp>

namespace robot_utils {

template <typename ValueType>
struct MultiplicationTraits{
  static ValueType scalarMultiplication(double multiplier, ValueType multiplicand){
    return multiplier * multiplicand;
  }
};

template <typename PrimType_>
struct MultiplicationTraits< kindr::TwistLinearVelocityLocalAngularVelocity< PrimType_ > >{
  using ValueType = kindr::TwistLinearVelocityLocalAngularVelocity< PrimType_ >;
  static ValueType scalarMultiplication(double multiplier, ValueType multiplicand){
    ValueType product;
    product.getTranslationalVelocity() = multiplier * multiplicand.getTranslationalVelocity();
    product.getRotationalVelocity() = multiplier * multiplicand.getRotationalVelocity();
    return product;
  }
};

template < typename PrimType_, typename Position_, typename Rotation_ >
struct MultiplicationTraits< kindr::HomogeneousTransformation<PrimType_, Position_, Rotation_ > >{
  using ValueType = kindr::HomogeneousTransformation<PrimType_, Position_, Rotation_ >;
  static_assert(!std::is_class<ValueType>::value, "HomogenousTransformations have no defined Multiplication Traits");
};

}  // robot_utils
