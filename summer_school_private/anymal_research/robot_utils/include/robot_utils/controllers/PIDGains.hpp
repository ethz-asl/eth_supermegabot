/*
 * PIDGains.hpp
 *
 *  Created on: Apr 27, 2018
 *      Author: Gabriel Hottiger
 */

#pragma once

// parameter_handler
#include <parameter_handler/Parameter.hpp>
#include <parameter_handler/helper_methods.hpp>

// Eigen
#include <Eigen/Core>

// STL
#include <type_traits>

namespace robot_utils {

template <typename T>
struct PIDGains {
  static_assert(parameter_handler::isTypeSupported<T>(), "[PIDGains] T is not supported by parameter_handler!");
  static_assert(!parameter_handler::traits::is_eigen_matrix<T>::value ||
                    parameter_handler::traits::is_fixed_size_eigen_matrix<T>::value,
                "[PIDGains] If T is an eigen type it must be of fixed size!");

  template <typename _T = T>
  explicit PIDGains(typename std::enable_if<std::is_arithmetic<_T>::value>::type* = 0)
      : proportionalGains_(
            parameter_handler::Parameter<_T>(_T{}, std::numeric_limits<_T>::lowest(), std::numeric_limits<_T>::max())),
        integralGains_(
            parameter_handler::Parameter<_T>(_T{}, std::numeric_limits<_T>::lowest(), std::numeric_limits<_T>::max())),
        derivativeGains_(
            parameter_handler::Parameter<_T>(_T{}, std::numeric_limits<_T>::lowest(), std::numeric_limits<_T>::max())),
        maxIntegral_(parameter_handler::Parameter<_T>(_T{}, std::numeric_limits<_T>::lowest(),
                                                      std::numeric_limits<_T>::max())) {}

  template <typename _T = T>
  explicit PIDGains(
      typename std::enable_if<parameter_handler::traits::is_fixed_size_eigen_matrix<_T>::value>::type* = 0)
      : proportionalGains_(parameter_handler::Parameter<_T>(
            _T::Zero(), std::numeric_limits<typename _T::Scalar>::lowest() * _T::Ones(),
            std::numeric_limits<typename _T::Scalar>::max() * _T::Ones())),
        integralGains_(parameter_handler::Parameter<_T>(_T::Zero(),
                                                        std::numeric_limits<typename _T::Scalar>::lowest() * _T::Ones(),
                                                        std::numeric_limits<typename _T::Scalar>::max() * _T::Ones())),
        derivativeGains_(parameter_handler::Parameter<_T>(
            _T::Zero(), std::numeric_limits<typename _T::Scalar>::lowest() * _T::Ones(),
            std::numeric_limits<typename _T::Scalar>::max() * _T::Ones())),
        maxIntegral_(parameter_handler::Parameter<_T>(_T::Zero(),
                                                      std::numeric_limits<typename _T::Scalar>::lowest() * _T::Ones(),
                                                      std::numeric_limits<typename _T::Scalar>::max() * _T::Ones())) {}

  template <typename U, typename = typename std::enable_if<
                            parameter_handler::is_parameter_of_type<typename std::decay<U>::type, T>::value> >
  PIDGains(U&& proportionalGains, U&& integralGains, U&& derivativeGains, U&& maxIntegral)
      : proportionalGains_(std::forward<U>(proportionalGains)),
        integralGains_(std::forward<U>(integralGains)),
        derivativeGains_(std::forward<U>(derivativeGains)),
        maxIntegral_(std::forward<U>(maxIntegral)) {}

  void resetToDefault() {
    proportionalGains_.resetToDefault();
    integralGains_.resetToDefault();
    derivativeGains_.resetToDefault();
    maxIntegral_.resetToDefault();
  }

  parameter_handler::Parameter<T> proportionalGains_;
  parameter_handler::Parameter<T> integralGains_;
  parameter_handler::Parameter<T> derivativeGains_;
  parameter_handler::Parameter<T> maxIntegral_;
};

using PIDGainsD = PIDGains<double>;
using PIDGains3D = PIDGains<Eigen::Vector3d>;

}  // namespace robot_utils