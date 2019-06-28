/*!
 * @file     type_traits.hpp
 * @author   Gabriel Hottiger
 * @date     Oct, 2017
 * @brief    Traits for the supported non-STL types.
 */

#pragma once

// Eigen
#include <Eigen/Core>

// STL
#include <type_traits>

namespace parameter_handler {

namespace traits {

//----------------------------------- EIGEN traits -------------------------------------//

//! isEigenMatrix false type
template<typename ValueType_, typename Enable_ = void>
struct is_eigen_matrix : std::false_type {};

//! isEigenMatrix true type
template<typename ValueType_>
struct is_eigen_matrix<ValueType_, typename std::enable_if< std::is_base_of< Eigen::MatrixBase<ValueType_>,
                                                            ValueType_ >::value>::type > : std::true_type {};

//! isEigenMartix (All matrices/vectors except Eigen::Vector3 types) false type
template<typename ValueType_, typename Enable_ = void>
struct is_fixed_size_eigen_matrix : std::false_type {};

//! isEigenMartix (All matrices/vectors except Eigen::Vector3 types) true type
template<typename ValueType_>
struct is_fixed_size_eigen_matrix<ValueType_, typename std::enable_if< is_eigen_matrix<ValueType_>::value &&
		ValueType_::RowsAtCompileTime != Eigen::Dynamic && ValueType_::ColsAtCompileTime != Eigen::Dynamic >::type > : std::true_type {};

} // end namespace traits

} // end namespace parameter_handler
