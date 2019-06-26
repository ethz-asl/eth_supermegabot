/*
 * filters.hpp
 *
 *  Created on: Jan 12, 2017
 *      Author: Dario Bellicoso, Philipp Leemann
 */

#pragma once

#include "robot_utils/filters/FirstOrderFilter.hpp"
#include "robot_utils/filters/FilterMovAvg.hpp"

#include <kindr/Core>
#include <Eigen/Core>

namespace robot_utils {

//! Define some helper types for filters.
using FirstOrderFilterD = FirstOrderFilter<double>;
using FirstOrderFilterF = FirstOrderFilter<float>;

using FilterMovAvgD = FilterMovAvg<double>;
using FilterMovAvgF = FilterMovAvg<float>;

using FirstOrderFilterEigenMatrix3d = FirstOrderFilter<Eigen::Matrix3d>;
using FirstOrderFilterEigenVector3d = FirstOrderFilter<Eigen::Vector3d>;

using FirstOrderFilterKindrVector3d = FirstOrderFilter<kindr::VectorTypeless3D>;
using FirstOrderFilterKindrPosition = FirstOrderFilter<kindr::Position3D>;
using FirstOrderFilterKindrLinearVelocity = FirstOrderFilter<kindr::Velocity3D>;
using FirstOrderFilterKindrAngularVelocity = FirstOrderFilter<kindr::LocalAngularVelocityD>;

template<typename ValueType, unsigned int Rows, unsigned int Cols>
using FirstOrderFilterEigenMatrixFixed = FirstOrderFilter<Eigen::Matrix<ValueType, Rows, Cols>>;

}
