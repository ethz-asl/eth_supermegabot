/*
 * typedefs.hpp
 *
 *  Created on: Oct, 2017
 *      Author: Philipp Leemann, Gabriel Hottiger
 */

#pragma once

#include "robot_utils/traits.hpp"

#include <vector>

namespace robot_utils {

template <typename T>
using VectorType = typename std::conditional<traits::is_eigen_matrix<T>::value,
        std::vector<T, Eigen::aligned_allocator<T>>, std::vector<T>>::type;

}
