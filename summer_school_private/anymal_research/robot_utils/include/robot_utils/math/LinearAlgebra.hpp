#pragma once

#include <Eigen/SVD>

namespace robot_utils {

Eigen::MatrixXd pseudoInverseAdaptiveDls(const Eigen::MatrixXd& A, double maxDampingFactor = 0.02, double singularRegionDimension = 0.06);

/**
 * Decomposes matrix A and thresholds the singular values to minimumSingularValueThreshold (ie changes singular or near-singular matrices to be nonsingular)
 * @param                             A: the matrix to be decomposed and adapted
 * @param minimumSingularValueThreshold: the lowest allowed value for the singular values (ie singular values lower than this are set to this)
 * @return the adapted matrix
 */
Eigen::MatrixXd adaptSingularValues(const Eigen::Ref<const Eigen::MatrixXd>& A, double minimumSingularValueThreshold);

} /* namespace robot_utils */
