/*
 * FilterMovAvgTest.cpp
 *
 *  Created on: Jan 11, 2017
 *      Author: Philipp Leemann
 */


// gtest
#include <gtest/gtest.h>

// robot utils
#include "robot_utils/filters/filters.hpp"

#include <Eigen/Core>

#include <array>
#include <algorithm> // std::fill(..)

TEST(FilterMovAvg, DoubleTest) {

  std::array<double, 400> input;
  std::array<double, 400> output;
  std::fill(input.begin(), std::next(input.begin(), 100), 1.0);
  std::fill(std::next(input.begin(), 100), input.end(), 3.0);

  robot_utils::FilterMovAvgD filter(200, input[0]);

  for (unsigned int k=0; k<input.size(); k++) {
      output[k] = filter.advance(input[k]);
  }


  ASSERT_TRUE(output[0] == 1.0);
  ASSERT_TRUE(output[99] == 1.0);
  ASSERT_TRUE(output[199] == 2.0);
  ASSERT_TRUE(output[299] == 3.0);
  ASSERT_TRUE(output[399] == 3.0);
}

TEST(FilterMovAvg, EigenInitTest) {

    robot_utils::FilterMovAvg<Eigen::Vector3d> filter(200);

    Eigen::Vector3d output = filter.advance(Eigen::Vector3d(0.0, 0.0, 0.0));

    ASSERT_TRUE(output == Eigen::Vector3d(0.0, 0.0, 0.0));
}

TEST(FilterMovAvg, EigenTest) {
  std::array<Eigen::Vector3d, 400> input;
  std::array<Eigen::Vector3d, 400> output;

  std::fill(input.begin(), std::next(input.begin(), 100), Eigen::Vector3d(0.0, 1.0, 2.0));
  std::fill(std::next(input.begin(), 100), input.end(), Eigen::Vector3d(2.0, 3.0, 4.0));

  robot_utils::FilterMovAvg<Eigen::Vector3d> filter(200, input[0]);

  for (unsigned int k=0; k<input.size(); k++) {
    output[k] = filter.advance(input[k]);
  }


  ASSERT_TRUE(output[0] == Eigen::Vector3d(0.0, 1.0, 2.0));
  ASSERT_TRUE(output[99] == Eigen::Vector3d(0.0, 1.0, 2.0));
  ASSERT_TRUE(output[199] == Eigen::Vector3d(1.0, 2.0, 3.0));
  ASSERT_TRUE(output[299] == Eigen::Vector3d(2.0, 3.0, 4.0));
  ASSERT_TRUE(output[399] == Eigen::Vector3d(2.0, 3.0, 4.0));

}