/*
 * type_macros.hpp
 *
 *  Created on: Dec 1, 2016
 *      Author: Gabriel Hottiger
 */

#pragma once

#include <Eigen/Core>

//-------------------------------------------------------------
//                DEFINE SUPPORTED TYPES
//-------------------------------------------------------------

/*!
 * Supported Integral Scalar Types
 */
#define PH_INTEGRAL_SCALAR_TYPES \
bool, \
char, \
short int, \
int, \
long int, \
unsigned short int

/*!
 *  Supported Integral Matrix Types
 */
#define PH_INTEGRAL_MATRIX_TYPES \
Eigen::Matrix3i, \
Eigen::MatrixXi, \
Eigen::Vector2i, \
Eigen::Vector3i, \
Eigen::Vector4i, \
Eigen::VectorXi

/*!
*  Supported Integral Types
*/
#define PH_INTEGRAL_TYPES \
PH_INTEGRAL_SCALAR_TYPES, \
PH_INTEGRAL_MATRIX_TYPES

/*!
 * Supported Floating Point Scalar Types
 */
#define PH_FLOATING_POINT_SCALAR_TYPES  \
double, \
float

/*!
 * Supported Floating Point Matrix Types
 */
#define PH_FLOATING_POINT_MATRIX_TYPES  \
Eigen::Matrix3f, \
Eigen::MatrixXf, \
Eigen::Vector2f, \
Eigen::Vector3f, \
Eigen::Vector4f, \
Eigen::VectorXf, \
Eigen::Matrix3d, \
Eigen::MatrixXd, \
Eigen::Vector2d, \
Eigen::Vector3d, \
Eigen::Vector4d, \
Eigen::VectorXd

/*!
* Supported Floating Point Types
*/
#define PH_FLOATING_POINT_TYPES \
PH_FLOATING_POINT_SCALAR_TYPES, \
PH_FLOATING_POINT_MATRIX_TYPES


/*!
* Supported Types
*/
#define PH_MATRIX_TYPES \
PH_INTEGRAL_MATRIX_TYPES, \
PH_FLOATING_POINT_MATRIX_TYPES

/*!
* Supported Types
*/
#define PH_SCALAR_TYPES \
PH_INTEGRAL_SCALAR_TYPES, \
PH_FLOATING_POINT_SCALAR_TYPES


/*!
* Supported Types
*/
#define PH_TYPES \
PH_MATRIX_TYPES, \
PH_SCALAR_TYPES


//-------------------------------------------------------------
