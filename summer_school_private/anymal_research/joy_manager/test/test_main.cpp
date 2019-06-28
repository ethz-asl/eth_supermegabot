/*!
* @file     test_main.cpp
* @author   Linus Isler
* @date     June, 2016
* @brief
*/


#include <gtest/gtest.h>
#include <ros/ros.h>

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  ros::init(argc, argv, "joy_manager_test");
  testing::InitGoogleTest(&argc, argv);
  srand((int)time(0));
  return RUN_ALL_TESTS();
}
