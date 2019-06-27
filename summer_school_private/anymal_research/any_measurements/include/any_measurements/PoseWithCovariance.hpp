#pragma once

#include "any_measurements/Pose.hpp"

#include <Eigen/Core>

namespace any_measurements {

struct PoseWithCovariance : public Pose
{
 public:
    using CovarianceMatrix = Eigen::Matrix<double, 6, 6>;

    PoseWithCovariance():
        Pose(),
        covariance_(CovarianceMatrix::Zero())
    {
    }

    PoseWithCovariance(
        const Time& time,
        const kindr::HomTransformQuatD& pose,
        const CovarianceMatrix& cov
    ):
        Pose(time, pose),
        covariance_(cov)
    {
    }


    ~PoseWithCovariance() override = default;

 public:
    CovarianceMatrix covariance_;
};

inline std::ostream& operator<<(std::ostream& os, const PoseWithCovariance& poseWithCovariance)
{
    return os << static_cast<Pose>(poseWithCovariance)
              << "\n Covariance" << poseWithCovariance.covariance_;
}

} /* namespace any_measurements */
