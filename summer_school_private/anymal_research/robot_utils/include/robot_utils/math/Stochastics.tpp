/*
 * Stochastics.tpp
 *
 *  Created on: Mar 24, 2016
 *      Author: Dario Bellicoso
 */

#pragma once

// boost
#include <boost/math/special_functions/pow.hpp>

// probabilistic utils
#include "robot_utils/math/Stochastics.hpp"

namespace robot_utils {

template<unsigned int N_>
double normalPdf(const Eigen::Matrix<double, N_, 1>& value, const Eigen::Matrix<double, N_, 1>& mean, const Eigen::Matrix<double, N_, N_>& cov) {
  const Eigen::Matrix<double, N_, 1> unbiasedValue = value - mean;
  return (std::exp(static_cast<double>(-0.5 * unbiasedValue.transpose() * cov.inverse() * unbiasedValue)) / (std::sqrt(boost::math::pow<N_>(2.0*M_PI) * cov.determinant())));
}

}
