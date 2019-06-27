/*
 * traits.hpp
 *
 *  Created on: Oct, 2017
 *      Author: Philipp Leemann, Gabriel Hottiger
 */

#pragma once

#include <Eigen/Core>
//#include <kindr/Core>

namespace robot_utils {
namespace traits {

//! isEigenMatrix false type
template<typename ValueType_, typename Enable_ = void>
struct is_eigen_matrix : std::false_type {};

//! isEigenMatrix true type
template<typename ValueType_>
struct is_eigen_matrix<ValueType_, typename std::enable_if<std::is_base_of < Eigen::MatrixBase < ValueType_>, ValueType_>::value>::type > : std::true_type {};


////! isKindrVector false type
//template<typename>
//struct is_kindr_vector : std::false_type {};
//
////! isKindrVector true type
//template<enum kindr::PhysicalType PhysicalType_, typename PrimType_, int Dimension_>
//struct is_kindr_vector<kindr::Vector<PhysicalType_,PrimType_, Dimension_>> : std::true_type {};

}
}