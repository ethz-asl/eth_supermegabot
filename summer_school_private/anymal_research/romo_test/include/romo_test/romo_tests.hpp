/*
 * romo_tests.hpp
 *
 *  Created on: Jun 30, 2017
 *      Author: dbellicoso
 */

#pragma once

// gtest
#include <gtest/gtest.h>

// kindr
#include "kindr/common/gtest_eigen.hpp"

// romo rbdl
#include "romo/common/phys_typedefs.hpp"

#define FIXTURE_TEST_TYPE MODEL_TEST_TYPE
#include "romo_test/KinematicsWorldToBody_tests.hpp"
#include "romo_test/DifferentialKinematicsWorldToPoint_tests.hpp"
#include "romo_test/DynamicsWorldToPoint_tests.hpp"
#include "romo_test/JacobianWorldToPoint_tests.hpp"
#include "romo_test/HessianWorldToPoint_tests.hpp"
#include "romo_test/RigidBody_tests.hpp"
#undef FIXTURE_TEST_TYPE
