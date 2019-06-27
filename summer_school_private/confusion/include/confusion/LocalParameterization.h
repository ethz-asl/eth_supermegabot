/*
 * Copyright 2018 Timothy Sandy
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef INCLUDE_CONFUSION_LOCALPARAMETERIZATION_H_
#define INCLUDE_CONFUSION_LOCALPARAMETERIZATION_H_

#include <vector>
#include <string>
#include <Eigen/Core>
#include <ceres/ceres.h>
#include "confusion/utilities/rotation_utils.h"
#include "confusion/utilities/distances.h"

namespace confusion {

//todo Could template on the local and global sizes to make the Jacobians fixed size

/**
 * @brief Define a local parameterization for a parameter block.
 *
 * A base class for parameter block local parameterizations. This extends the
 * LocalParameterization of Ceres (see the relevent section of their documentation)
 * to also include a boxMinus function for use to express the change in parameters
 * in local space in the PriorCost. See QuatParam and its unit tests for an
 * example implementation and usage.
 */
class LocalParameterizationBase : public ceres::LocalParameterization {
 public:
  /**
   * @brief Computes the distance between two parameter values in local (tangent) space.
   * @param xi Left-hand-side parameter
   * @param xj Right-hand-side parameter
   * @param xi_minus_xj LHS [boxminus] RHS
   */
  virtual void boxMinus(const double *xi, const double *xj,
                        double *xi_minus_xj) const = 0;

  /**
   * @brief Computes the derivative of the boxminus operation with respect to the left-hand-side parameter
   * @param xi Left-hand-side parameter
   * @param xj Right-hand-side parameter
   * @return The derivative of LHS [boxminus] RHS with repsect to LHS parameter
   */
  virtual Eigen::MatrixXd boxMinusJacobianLeft(double const *xi, double const *xj) const = 0;

  /**
   * @brief Computes the derivative of the boxminus operation with respect to the right-hand-side parameter
   * @param xi Left-hand-side parameter
   * @param xj Right-hand-side parameter
   * @return The derivative of LHS [boxminus] RHS with repsect to RHS parameter
   */
  virtual Eigen::MatrixXd boxMinusJacobianRight(double const *xi, double const *xj) const = 0;
};

class QuatParam : public LocalParameterizationBase {
 public:
  ~QuatParam() {}

  bool Plus(const double *x_raw, const double *delta_raw, double *x_plus_delta_raw) const {
    const Eigen::Quaterniond x(x_raw);
    const Eigen::Vector3d delta(delta_raw);

    Eigen::Map<Eigen::Quaterniond> x_plus_delta(x_plus_delta_raw);

    x_plus_delta = quaternionBoxPlus<double>(x, delta);

    return true;
  }

  bool ComputeJacobian(const double *x, double *jacobian) const {
    Eigen::Quaterniond q(x);
    Eigen::Map<Eigen::Matrix<double, 4, 3, Eigen::RowMajor> > dq_ddq(jacobian);

    dq_ddq = quaternionBoxPlusJacob(q);

    return true;
  }

  int GlobalSize() const { return 4; }
  int LocalSize() const { return 3; }

  //todo There is a minus factor mismatch between the boxPlus and boxMinus, though that shouldn't impact solver performance
  //todo Make the rotation library functions take raw pointers?
  void boxMinus(double const *xi, double const *xj, double *dist) const {
    const Eigen::Quaterniond qi(xi);
    const Eigen::Quaterniond qj(xj);
//std::cout << "qi: " << qi.coeffs().transpose() << "\nqj: " << qj.coeffs().transpose() << std::endl;
    QuatDistance(qi, qj, dist);
  }

  Eigen::MatrixXd boxMinusJacobianLeft(double const *xi, double const *xj) const {
    const Eigen::Quaterniond qj(xj);

    //Derivative in [x,y,z,w] column order for generality in confusion
    //todo Duplicate of calc_de_dq in rotation_utils
    Eigen::MatrixXd j(3, 4);
    j << 	qj.w(), -qj.z(),  qj.y(), -qj.x(),
        qj.z(),  qj.w(), -qj.x(), -qj.y(),
        -qj.y(),  qj.x(),  qj.w(), -qj.z();
        j *= 2.0;

    return j;
  }

  //todo Could make this statically sized via templating?
  Eigen::MatrixXd boxMinusJacobianRight(double const *xi, double const *xj) const {
    const Eigen::Quaterniond qi(xi);

    //Derivative in [x,y,z,w] column order for generality in confusion
    //todo Duplicate of calc_de_dq in rotation_utils
    Eigen::MatrixXd j(3, 4);
    j << -qi.w(), qi.z(), -qi.y(), qi.x(),
        -qi.z(), -qi.w(), qi.x(), qi.y(),
        qi.y(), -qi.x(), -qi.w(), qi.z();
    j *= 2.0;

    return j;
  }
};

class FixedYawParameterization : public LocalParameterizationBase {
 public:
  ~FixedYawParameterization() {}

  bool Plus(const double *x_raw, const double *delta_raw, double *x_plus_delta_raw) const {
    //q' = dqx * dqy * q = dq * q

    const Eigen::Map<const Eigen::Quaterniond> q_in(x_raw);

    double phi_x = delta_raw[0] / 2.0;
    double phi_y = delta_raw[1] / 2.0;
    Eigen::Quaterniond dqx(cos(phi_x), sin(phi_x), 0.0, 0.0);
    Eigen::Quaterniond dqy(cos(phi_y), 0.0, sin(phi_y), 0.0);

    Eigen::Map<Eigen::Quaterniond> x_plus_delta(x_plus_delta_raw);

    x_plus_delta = dqx * dqy * q_in;

    return true;
  }

  bool ComputeJacobian(const double *x, double *jacobian) const {
    const Eigen::Map<const Eigen::Quaterniond> q_in(x);

    //This is the 2nd and 3rd cols of QuatProduct_jacob_left
    //The Jacobian is given in [x,y,z,w] row order.
    Eigen::Map<Eigen::Matrix<double, 4, 2, Eigen::RowMajor> > dq_dp(jacobian);
    dq_dp << q_in.w(), q_in.z(),
        -q_in.z(), q_in.w(),
        q_in.y(), -q_in.x(),
        -q_in.x(), -q_in.y();
    dq_dp *= 0.5;

    return true;
  }

  int GlobalSize() const { return 4; }
  int LocalSize() const { return 2; }

  void boxMinus(double const *xi, double const *xj, double *dist) const {
    const Eigen::Quaterniond qi(xi);
    const Eigen::Quaterniond qj(xj);

    Eigen::Vector3d dist_;
    QuatDistance(qj, qi, dist_.data());
//    	QuatDistance(qi.conjugate(), qj.conjugate(), dist_.data());
//std::cout << "FixedYaw boxMinus dist: " << dist_.transpose() << std::endl;
//    	if (fabs(dist_(2)) > 1e-10) //Why does this threshold have to be so high? The locality of QuatDistance?
//    		std::cout << "ERROR: FixedYaw boxMinus gave a non-zero distance in the z-direction!?" << std::endl;

    dist[0] = dist_(0);
    dist[1] = dist_(1);
  }

  Eigen::MatrixXd boxMinusJacobianLeft(double const *xi, double const *xj) const {
    const Eigen::Quaterniond qj(xj);

    //The derivative is the top two columns of QuatParam->boxMinusJacobianRight (the boxMinus operator flips the order to be consistent with the boxPlus operator)
    //todo Use a common utility function?
    Eigen::MatrixXd j(2, 4);
//		j << 	-qi.w(), qi.z(), -qi.y(), qi.x(),
//				-qi.z(), -qi.w(), qi.x(), qi.y();

    //Negated x,y,z because of the conjugates added above
//		j << 	-qi.w(), -qi.z(), qi.y(), -qi.x(),
//				qi.z(), -qi.w(), -qi.x(), -qi.y();
//		j << 	qi.w(), qi.z(), -qi.y(), -qi.x(),
//				-qi.z(), qi.w(), qi.x(), -qi.y();

    j << -qj.w(), qj.z(), -qj.y(), qj.x(),
        -qj.z(), -qj.w(), qj.x(), qj.y();
    j *= 2.0;

    return j;
  }

  Eigen::MatrixXd boxMinusJacobianRight(double const *xi, double const *xj) const {
    const Eigen::Quaterniond qi(xi);

    //The derivative is the top two columns of QuatParam->boxMinusJacobianLeft (the boxMinus operator flips the order to be consistent with the boxPlus operator)
    //todo Use a common utility function?
    Eigen::MatrixXd j(2, 4);
//		j << 	-qi.w(), qi.z(), -qi.y(), qi.x(),
//				-qi.z(), -qi.w(), qi.x(), qi.y();

    //Negated x,y,z because of the conjugates added above
//		j << 	-qi.w(), -qi.z(), qi.y(), -qi.x(),
//				qi.z(), -qi.w(), -qi.x(), -qi.y();
//		j << 	qi.w(), qi.z(), -qi.y(), -qi.x(),
//				-qi.z(), qi.w(), qi.x(), -qi.y();

    j << qi.w(), -qi.z(), qi.y(), -qi.x(),
        qi.z(), qi.w(), -qi.x(), -qi.y();
    j *= 2.0;

    return j;
  }
};

} // namespace confusion

#endif /* INCLUDE_CONFUSION_LOCALPARAMETERIZATION_H_ */
