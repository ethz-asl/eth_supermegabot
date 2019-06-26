/*
 * typedefs.hpp
 *
 *  Created on: Nov 10, 2015
 *      Author: Dario Bellicoso
 */

#pragma once

// kindr
#include <kindr/Core>

namespace romo {
namespace internal {

template <unsigned int N_ = 0>
using GeneralizedCoordinates = typename Eigen::Matrix<double, N_, 1>;

template <unsigned int N_ = 0>
using GeneralizedVelocities = typename Eigen::Matrix<double, N_, 1>;

template <unsigned int N_ = 0>
using GeneralizedAccelerations = typename Eigen::Matrix<double, N_, 1>;

template <unsigned int N_ = 0>
using JointPositions = typename kindr::Position<double, N_>;

template <unsigned int N_ = 0>
using JointVelocities = typename kindr::Velocity<double, N_>;

template <unsigned int N_ = 0>
using JointAccelerations = typename kindr::Acceleration<double, N_>;

template <unsigned int N_ = 0>
using JointTorques = typename kindr::Torque<double, N_>;

} // namespace internal
} // namespace romo
