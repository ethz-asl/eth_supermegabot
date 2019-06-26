/*
 * get_param_test.cpp
 *
 *  Created on: Jan 11, 2017
 *      Author: Christian Gehring
 */

// gtest
#include <gtest/gtest.h>

// ros
#include <ros/ros.h>

// param io
#include <param_io/get_param.hpp>

TEST(GetParam, getParam_double) {
  ros::NodeHandle nh("~");

  const double testdefault = 3.0;
  double test = testdefault;
  ASSERT_FALSE(param_io::getParam(nh, "double", test));
  ASSERT_EQ(test, testdefault);
}

TEST(GetParam, getParam_bool) {
  ros::NodeHandle nh("~");

  const bool testdefault = true;
  bool test = testdefault;
  ASSERT_FALSE(param_io::getParam(nh, "double", test));
  ASSERT_EQ(test, testdefault);
}

TEST(GetParam, param_double) {
  ros::NodeHandle nh("~");
  double testdouble = 0.0;

  // Test with const default.
  testdouble = 1.0;
  const double testdefaultconst = 2.0;
  testdouble = param_io::param(nh, "double", testdefaultconst);
  ASSERT_EQ(testdefaultconst, testdouble);

  // Test with non-const default.
  testdouble = 1.0;
  double testdefault = 100.0;
  testdouble = param_io::param(nh, "double", testdefault);
  ASSERT_EQ(testdefault, testdouble);
}

TEST(GetParam, param_bool) {
  ros::NodeHandle nh("~");
  bool testbool = true;

  // Test with const default.
  testbool = true;
  const bool testdefaultconst = false;
  testbool = param_io::param(nh, "bool", testdefaultconst);
  ASSERT_EQ(testdefaultconst, testbool);

  // Test with non-const default.
  testbool = true;
  bool testdefault = false;
  testbool = param_io::param(nh, "bool", testdefault);
  ASSERT_EQ(testdefault, testbool);
}
