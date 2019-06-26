/*
 * DifferenceTraits.hpp
 *
 *  Created on: Feb 12, 2019
 *      Author: Perry Franklin
 */

#pragma once

#include <kindr/Core>
#include <robot_utils/math/DerivativeTraits.hpp>

namespace robot_utils {

template <typename ValueType>
struct DifferenceTraits{
  using ReturnType = ValueType;
  static ReturnType subtract(const ValueType minuend, const ValueType& subtrahend){
      return minuend - subtrahend;
    }
};

template < typename PrimType_, typename Position_, typename Rotation_ >
struct DifferenceTraits< kindr::HomogeneousTransformation<PrimType_, Position_, Rotation_ > >{
  using ValueType = kindr::HomogeneousTransformation<PrimType_, Position_, Rotation_ >;
  using ReturnType = typename DerivativeTraits<ValueType>::type;
  using LinearVelocity = typename ReturnType::PositionDiff;
  using RotationVelocity = typename ReturnType::RotationDiff;

  static ReturnType subtract(const ValueType minuend, const ValueType& subtrahend){
    ReturnType difference;
    difference.getTranslationalVelocity() = static_cast<LinearVelocity>(minuend.getPosition() - subtrahend.getPosition());
    difference.getRotationalVelocity() = static_cast<RotationVelocity>(minuend.getRotation().boxMinus(subtrahend.getRotation()));
    return difference;
  }
};

}  // robot_utils
