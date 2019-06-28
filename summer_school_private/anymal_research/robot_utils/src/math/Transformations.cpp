/*
 * Transformations.hpp
 *
 *  Created on: May 23, 2016
 *      Author: Christian Gehring
 */

#include "robot_utils/math/Transformations.hpp"
#include <iostream>
#include <Eigen/Dense>

namespace robot_utils {

/*
 * Inspired by OpenCV 2.4.13:
 *
 * Calculates coefficients of perspective transformation
 * which maps (xi,yi) to (ui,vi), (i=1,2,3,4):
 *
 *      c00*xi + c01*yi + c02
 * ui = ---------------------
 *      c20*xi + c21*yi + c22
 *
 *      c10*xi + c11*yi + c12
 * vi = ---------------------
 *      c20*xi + c21*yi + c22
 *
 * Coefficients are calculated by solving linear system:
 * / x0 y0  1  0  0  0 -x0*u0 -y0*u0 \ /c00\ /u0\
 * | x1 y1  1  0  0  0 -x1*u1 -y1*u1 | |c01| |u1|
 * | x2 y2  1  0  0  0 -x2*u2 -y2*u2 | |c02| |u2|
 * | x3 y3  1  0  0  0 -x3*u3 -y3*u3 |.|c10|=|u3|,
 * |  0  0  0 x0 y0  1 -x0*v0 -y0*v0 | |c11| |v0|
 * |  0  0  0 x1 y1  1 -x1*v1 -y1*v1 | |c12| |v1|
 * |  0  0  0 x2 y2  1 -x2*v2 -y2*v2 | |c20| |v2|
 * \  0  0  0 x3 y3  1 -x3*v3 -y3*v3 / \c21/ \v3/
 *
 * where:
 *   cij - matrix coefficients, c22 = 1
 */
Eigen::Matrix3d getPerspectiveTransform( const std::vector<Eigen::Vector3d>& src,
                                         const std::vector<Eigen::Vector3d>& dst )
{
    assert(src.size() == 4);
    assert(dst.size() == 4);

    Eigen::Matrix3d M;
    Eigen::Matrix<double, 8, 1> x;
    Eigen::Matrix<double, 8, 1> b;
    Eigen::Matrix<double, 8, 8> A;

    for( int i = 0; i < 4; ++i )
    {
      A(i,0) = A(i+4,3) = src[i].x();
      A(i,1) = A(i+4,4) = src[i].y();
      A(i,2) = A(i+4,5) = 1;
      A(i,3) = A(i,4) = A(i,5) =
      A(i+4,0) = A(i+4,1) = A(i+4,2) = 0;
      A(i,6) = -src[i].x()*dst[i].x();
      A(i,7) = -src[i].y()*dst[i].x();
      A(i+4,6) = -src[i].x()*dst[i].y();
      A(i+4,7) = -src[i].y()*dst[i].y();
      b(i) = dst[i].x();
      b(i+4) = dst[i].y();
    }

    x = A.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(b);
    M(0,0) = x(0);
    M(0,1) = x(1);
    M(0,2) = x(2);
    M(1,0) = x(3);
    M(1,1) = x(4);
    M(1,2) = x(5);
    M(2,0) = x(6);
    M(2,1) = x(7);
    M(2,2) = 1.0;

    return M;
}


/* Inspired by OpenCV 2.4.13:
 *
 * Calculates coefficients of affine transformation
 * which maps (xi,yi) to (ui,vi), (i=1,2,3):
 *
 * ui = c00*xi + c01*yi + c02
 *
 * vi = c10*xi + c11*yi + c12
 *
 * Coefficients are calculated by solving linear system:
 * / x0 y0  1  0  0  0 \ /c00\ /u0\
 * | x1 y1  1  0  0  0 | |c01| |u1|
 * | x2 y2  1  0  0  0 | |c02| |u2|
 * |  0  0  0 x0 y0  1 | |c10| |v0|
 * |  0  0  0 x1 y1  1 | |c11| |v1|
 * \  0  0  0 x2 y2  1 / |c12| |v2|
 *
 * where:
 *   cij - matrix coefficients
 */

Eigen::Matrix3d getAffineTransform( const std::vector<Eigen::Vector3d>& src,
                                    const std::vector<Eigen::Vector3d>& dst )
{
  assert(src.size() == 3);
  assert(dst.size() == 3);

  Eigen::Matrix3d M;
  Eigen::Matrix<double, 6, 1> x;
  Eigen::Matrix<double, 6, 1> b;
  Eigen::Matrix<double, 6, 6> A;

  for( int i = 0; i < 3; i++ )
  {
      A(i, 0) = A(i+3, 3) = src[i].x();
      A(i, 1) = A(i+3, 4) = src[i].y();
      A(i, 2) = A(i+3, 5) = 1;
      A(i, 3) = A(i, 4) = A(i, 5) = 0;
      A(i+3, 0) = A(i+3, 1) = A(i+3, 2) = 0;
      b(i) = dst[i].x();
      b(i+3) = dst[i].y();
  }

  x = A.partialPivLu().solve(b);

  M(0,0) = x(0);
  M(0,1) = x(1);
  M(0,2) = x(2);
  M(1,0) = x(3);
  M(1,1) = x(4);
  M(1,2) = x(5);
  M(2,0) = 0.0;
  M(2,1) = 0.0;
  M(2,2) = 1.0;

  return M;
}

} // namespace
