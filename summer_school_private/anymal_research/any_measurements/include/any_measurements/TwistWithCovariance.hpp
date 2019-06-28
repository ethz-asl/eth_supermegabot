#pragma once

#include "any_measurements/Twist.hpp"

#include <Eigen/Core>

namespace any_measurements {

struct TwistWithCovariance : public Twist
{
 public:
    using CovarianceMatrix = Eigen::Matrix<double, 6, 6>;

    TwistWithCovariance():
        Twist(),
        covariance_(CovarianceMatrix::Zero())
    {
    }

    TwistWithCovariance(
        const Time& time,
        const kindr::TwistLocalD& twist,
        const CovarianceMatrix& cov
    ):
        Twist(time, twist),
        covariance_(cov)
    {
    }

    ~TwistWithCovariance() override = default;

 public:
    CovarianceMatrix covariance_;
};

inline std::ostream& operator<<(std::ostream& os, const TwistWithCovariance& twistWithCovariance)
{
    return os << static_cast<Twist>(twistWithCovariance)
              << "\n Covariance: " << twistWithCovariance.covariance_;
}

} /* namespace any_measurements */
