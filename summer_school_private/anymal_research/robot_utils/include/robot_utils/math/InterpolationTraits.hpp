/*
 * InterpolationTraits.hpp
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
struct InterpolationTraits{
  using DifferenceType = typename DerivativeTraits<ValueType>::type;
  static_assert(std::is_same<ValueType, DifferenceType>::value, "By default, InterpolationTraits assumes that the DifferenceType (as defined by DerivativeTraits<ValueType>) is the same as the ValueType. If this is not the case, you probably need to define a speicif");
  static ValueType applyInterpolation(double s, ValueType initialValue, DifferenceType totalDifference){
    return initialValue + (s * totalDifference);
  }
};

template < typename PrimType_, typename Position_, typename Rotation_ >
struct InterpolationTraits< kindr::HomogeneousTransformation<PrimType_, Position_, Rotation_ > >{
  using ValueType = kindr::HomogeneousTransformation<PrimType_, Position_, Rotation_ >;
  using DifferenceType = typename DerivativeTraits<ValueType>::type;

  static ValueType applyInterpolation(double s, ValueType initialValue, DifferenceType totalDifference){
    ValueType interpolatedValue;
    interpolatedValue.getPosition() = initialValue.getPosition() + s * static_cast<Position_>(totalDifference.getTranslationalVelocity());
    interpolatedValue.getRotation() = initialValue.getRotation().boxPlus(s * totalDifference.getRotationalVelocity().toImplementation());
    return interpolatedValue;
  }
};

}  // robot_utils
