/*!
 * @file    ImuWithCovariance.hpp
 * @author  Philipp Leemann
 * @date    Sep 19, 2016
 * @version 0.0
 *
 */

#pragma once

#include <Eigen/Core>

#include "any_measurements/Imu.hpp"


namespace any_measurements {

struct ImuWithCovariance : public Imu
{
public:
    using CovarianceMatrix = Eigen::Matrix<double, 3, 3>;

    ImuWithCovariance():
            Imu(),
            orientationCovariance_(CovarianceMatrix::Zero()),
            angularVelocityCovariance_(CovarianceMatrix::Zero()),
            linearAccelerationCovariance_(CovarianceMatrix::Zero())
    {
    }

    ImuWithCovariance(const Time& time,
        const kindr::RotationQuaternionD& orientation,
        const kindr::LocalAngularVelocityD& angularVelocity,
        const kindr::Acceleration3D& linearAcceleration,
        const CovarianceMatrix& orientationCovariance,
        const CovarianceMatrix& angularVelocityCovariance,
        const CovarianceMatrix& linearAccelerationCovariance
    ):
            Imu(time, orientation, angularVelocity, linearAcceleration),
            orientationCovariance_(orientationCovariance),
            angularVelocityCovariance_(angularVelocityCovariance),
            linearAccelerationCovariance_(linearAccelerationCovariance)
    {
    }

    ~ImuWithCovariance() override = default;

public:
    CovarianceMatrix orientationCovariance_;
    CovarianceMatrix angularVelocityCovariance_;
    CovarianceMatrix linearAccelerationCovariance_;
};

inline std::ostream& operator<<(std::ostream& os, const ImuWithCovariance& imuWithCovariance)
{
    return os << static_cast<Imu>(imuWithCovariance)
              << "\n Orientation Covariance: " << imuWithCovariance.orientationCovariance_
              << "\n Linear Acceleration Covariance: " << imuWithCovariance.linearAccelerationCovariance_
              << "\n Angular Velocity Covariance: " << imuWithCovariance.angularVelocityCovariance_;
}

} /* namespace any_measurements */
