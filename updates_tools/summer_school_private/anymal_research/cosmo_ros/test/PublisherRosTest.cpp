/*
 * PublisherRosTest.cpp
 *
 *  Created on: Jun 29, 2017
 *      Author: gehrinch
 */


#include <gtest/gtest.h>
#include <ros/ros.h>
#include <cosmo_ros/cosmo_ros.hpp>

#include "UnitTestShmMsg.hpp"

namespace cosmo_ros {


class TestObject {
 public:
  void callback(const UnitTestShmMsg& msg) {

  }
};

} // namespace

TEST(PublisherRosTest, helper_methods)
{
  using namespace ros;
  using namespace cosmo_ros;
  NodeHandle nodeHandle("~");

  using MsgShm = UnitTestShmMsg;
  using MsgRos = UnitTestRosMsg;

  auto pub = cosmo_ros::advertiseShmRos<MsgShm, MsgRos, ConversionTraits>(nodeHandle, "test", "/test");


  auto options = std::make_shared<cosmo_ros::PublisherRosOptions>("test", nodeHandle);
  auto optionsPub = cosmo_ros::advertiseShmRos<MsgShm,MsgRos,ConversionTraits>(options);


  auto defaultOptions = std::make_shared<cosmo_ros::PublisherRosOptions>("test", nodeHandle);
  defaultOptions->topic_ = "haha";
  defaultOptions->rosLatch_ =  false;
  defaultOptions->rosQueueSize_ = 1;
  auto defaultPub = cosmo_ros::advertiseShmRos<MsgShm,MsgRos,ConversionTraits>("mypublisher", defaultOptions);

  ASSERT_EQ(std::string{"/cosmo_ros_publisher_ros/my_topic"}, defaultOptions->topic_);
  ASSERT_EQ(true, defaultOptions->rosLatch_);
  ASSERT_EQ(99u, defaultOptions->rosQueueSize_);

}


TEST(SubscriberRosTest, helper_methods)
{
  using namespace ros;
  using namespace cosmo_ros;
  NodeHandle nodeHandle("~");

  using MsgShm = UnitTestShmMsg;
  using MsgRos = UnitTestRosMsg;

  TestObject testObj;


  auto defaultOptions = std::make_shared<cosmo_ros::SubscriberRosOptions<MsgShm>>("/test", std::bind(&TestObject::callback, &testObj, std::placeholders::_1), nodeHandle);
  defaultOptions->topic_ = "haha";
  defaultOptions->rosQueueSize_ = 1;

  auto defaultPub = cosmo_ros::subscribeShmRos<MsgShm, MsgRos, ConversionTraits>("mysubscriber", defaultOptions);

  ASSERT_EQ(std::string{"/cosmo_ros_publisher_ros/my_topic"}, defaultOptions->topic_);
  ASSERT_EQ(77u, defaultOptions->rosQueueSize_);


}
