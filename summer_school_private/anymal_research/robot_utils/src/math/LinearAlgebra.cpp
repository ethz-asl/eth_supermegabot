/*
 * LinearAlgebra.cpp
 *
 *  Created on: Aug 22, 2013
 *      Author: Péter Fankhauser, Dario Bellicoso
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

// robot utils
#include "robot_utils/math/LinearAlgebra.hpp"

// stl
#include <iostream>

namespace robot_utils {

Eigen::MatrixXd pseudoInverseAdaptiveDls(const Eigen::MatrixXd& A, double maxDampingFactor, double singularRegionDimension) {
  Eigen::JacobiSVD<Eigen::MatrixXd> Asvd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const Eigen::VectorXd singularValues = Asvd.singularValues();

  const double minimumSingularValue = singularValues(Asvd.nonzeroSingularValues()-1);

  double dampingFactor = 0.0;
  if (minimumSingularValue < singularRegionDimension) {
    dampingFactor = (1.0 - (minimumSingularValue/singularRegionDimension)*(minimumSingularValue/singularRegionDimension)) * maxDampingFactor;
  }

  Eigen::VectorXd singularValuesInverted = singularValues;
  for (int k=0; k<singularValues.rows(); ++k) {
    singularValuesInverted(k) = singularValues(k)/(singularValues(k)*singularValues(k) + dampingFactor*dampingFactor);
  }

  return Asvd.matrixV()*singularValuesInverted.asDiagonal()*Asvd.matrixU().transpose();
}

Eigen::MatrixXd adaptSingularValues(const Eigen::Ref<const Eigen::MatrixXd>& A, double minimumSingularValueThreshold) {

  const Eigen::JacobiSVD<Eigen::MatrixXd> Asvd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::VectorXd singularValues = Asvd.singularValues();

  for (unsigned int i = 0; i < singularValues.size(); ++i ){
    if (singularValues(i) < minimumSingularValueThreshold){
      singularValues(i) = minimumSingularValueThreshold;
    }
  }

  return Asvd.matrixU()*singularValues.asDiagonal()*Asvd.matrixV().transpose();

}

} /* namespace robot_utils */
