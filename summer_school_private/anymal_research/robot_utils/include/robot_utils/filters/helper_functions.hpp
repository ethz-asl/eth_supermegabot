/*
 * helper_functions.hpp
 *
 *  Created on: Oct, 2017
 *      Author: Philipp Leemann
 */

#pragma once

#include "robot_utils/traits.hpp"

namespace robot_utils {

// default type
template<typename ValueType>
constexpr ValueType getDefaultValue(typename std::enable_if<   !std::is_arithmetic<ValueType>::value
                                                            && !traits::is_eigen_matrix<ValueType>::value>::type * = 0)
{
    return ValueType();
}

// arithmetic type
template<typename ValueType>
constexpr ValueType getDefaultValue(typename std::enable_if<std::is_arithmetic<ValueType>::value>::type * = 0)
{
    return ValueType(0);
}

// eigen type
template<typename ValueType>
constexpr ValueType getDefaultValue(typename std::enable_if<traits::is_eigen_matrix<ValueType>::value>::type * = 0)
{
    static_assert(ValueType::RowsAtCompileTime != Eigen::Dynamic, "Cannot create filter using dynamic sized Eigen matrix without providing default values!");
    return ValueType::Zero(ValueType::RowsAtCompileTime, ValueType::ColsAtCompileTime);
}



}