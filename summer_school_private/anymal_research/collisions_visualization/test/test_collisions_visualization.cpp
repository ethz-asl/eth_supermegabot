/*
 *  Created on: Aug 3, 2017
 *      Author: Perry Franklin
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#include <gtest/gtest.h>

// ROS
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "collisions_visualization_test");
  testing::InitGoogleTest(&argc, argv);
  srand((int) time(0));
  return RUN_ALL_TESTS();
}
