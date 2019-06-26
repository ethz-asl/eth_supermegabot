/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2015 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#ifndef RBDL_MATH_H
#define RBDL_MATH_H

#include "any_rbdl/rbdl_config.h"

#ifdef ANY_RBDL_USE_SIMPLE_MATH
	#include "any_rbdl/SimpleMath/SimpleMathFixed.h"
	#include "any_rbdl/SimpleMath/SimpleMathDynamic.h"
	#include "any_rbdl/SimpleMath/SimpleMathMixed.h"
	#include "any_rbdl/SimpleMath/SimpleMathQR.h"
	#include "any_rbdl/SimpleMath/SimpleMathCholesky.h"
	#include "any_rbdl/SimpleMath/SimpleMathCommaInitializer.h"
	#include <vector>

	typedef SimpleMath::Fixed::Matrix<double, 3,1> Vector3_t;
	typedef SimpleMath::Fixed::Matrix<double, 3,3> Matrix3_t;
	typedef SimpleMath::Fixed::Matrix<double, 4,1> Vector4_t;

	typedef SimpleMath::Fixed::Matrix<double, 6,1> SpatialVector_t;
	typedef SimpleMath::Fixed::Matrix<double, 6,6> SpatialMatrix_t;

	typedef SimpleMath::Fixed::Matrix<double, 6,3> Matrix63_t;

	typedef SimpleMath::Dynamic::Matrix<double> MatrixN_t;
	typedef SimpleMath::Dynamic::Matrix<double> VectorN_t;

#else
	#include <Eigen/Dense>
	#include <Eigen/StdVector>
	#include <Eigen/QR>

	#include "any_rbdl/rbdl_eigenmath.h"

	typedef Eigen::Matrix<double, 6, 3> Matrix63_t;

	typedef Eigen::VectorXd VectorN_t;
	typedef Eigen::MatrixXd MatrixN_t;
#endif

namespace RigidBodyDynamics {

/** \brief Math types such as vectors and matrices and utility functions. */
namespace Math {
	typedef Vector3_t Vector3d;
	typedef Vector4_t Vector4d;
	typedef Matrix3_t Matrix3d;
	typedef SpatialVector_t SpatialVector;
	typedef SpatialMatrix_t SpatialMatrix;
	typedef Matrix63_t Matrix63;
	typedef VectorN_t VectorNd;
	typedef MatrixN_t MatrixNd;
} /* Math */

} /* RigidBodyDynamics */

#include "any_rbdl/Quaternion.h"
#include "any_rbdl/SpatialAlgebraOperators.h"

// If we use Eigen3 we have to create specializations of the STL
// std::vector such that the alignment is done properly.
#ifndef ANY_RBDL_USE_SIMPLE_MATH
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(RigidBodyDynamics::Math::SpatialVector)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(RigidBodyDynamics::Math::SpatialMatrix)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(RigidBodyDynamics::Math::Matrix63)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(RigidBodyDynamics::Math::SpatialTransform)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(RigidBodyDynamics::Math::SpatialRigidBodyInertia)
#endif

/* RBDL_MATH_H_H */
#endif
