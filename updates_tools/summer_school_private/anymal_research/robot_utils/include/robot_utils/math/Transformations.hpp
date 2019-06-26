/*
 * Transformations.hpp
 *
 *  Created on: May 23, 2016
 *      Author: Christian Gehring
 */
#pragma once

#include <vector>
#include <Eigen/SVD>


namespace robot_utils {

/*!  Estimates the perspective transformation matrix (homography)
 *
 * Calculates coefficients of perspective transformation
 * which maps (xi,yi) to (ui,vi), (i=1,2,3,4):
 *
 * The function calculates the 3 \times 3 matrix of a perspective transform so that:
 *
 *  \begin{bmatrix} t_i x'_i \\ t_i y'_i \\ t_i \end{bmatrix} = \texttt{map\_matrix} \cdot \begin{bmatrix} x_i \\ y_i \\ 1 \end{bmatrix}
 *
 * where
 *
 * dst(i)=(x'_i,y'_i), src(i)=(x_i, y_i), i=0,1,2,3
 *
 * @param src   list of four source points src(i)=(x_i, y_i, 1.0), i=0,1,2,3
 * @param dst   list of four destination points dst(i)=(x_i, y_i, 1.0), i=0,1,2,3
 * @return transformation matrix
 */
Eigen::Matrix3d getPerspectiveTransform( const std::vector<Eigen::Vector3d>& src,
                                         const std::vector<Eigen::Vector3d>& dst );

/*! Estimates the affine transformation matrix
 *
 * @param src list of three source points src(i)=(x_i, y_i, 1.0), i=0,1,2
 * @param dst list of three destination points dst(i)=(x_i, y_i, 1.0), i=0,1,2
 * @return transformation matrix
 */
Eigen::Matrix3d getAffineTransform( const std::vector<Eigen::Vector3d>& src,
                                    const std::vector<Eigen::Vector3d>& dst );

} // namespace
