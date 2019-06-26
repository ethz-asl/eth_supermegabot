/*!
 * @file    ConversionTraitsTest.cpp
 * @author  Philipp Leemann
 * @date    Jan, 2018
 * @version 0.0
 */

#include <gtest/gtest.h>

#include "any_measurements_ros/ConversionTraits.hpp"

#include <array>

template<typename ShmMsg, typename RosMsg>
void doConversion(ShmMsg& shmMsg, RosMsg& rosMsg)
{
    // need to convert TO ros first since rosMsg is resized during that conversion
    any_measurements_ros::ConversionTraits<ShmMsg, RosMsg>::convert(shmMsg, rosMsg);
    any_measurements_ros::ConversionTraits<ShmMsg, RosMsg>::convert(rosMsg, shmMsg);

    // the functions below should do the same as the functions above, test them nonetheless to check for interface existence
    any_measurements_ros::toRos(shmMsg, rosMsg);
    any_measurements_ros::fromRos(rosMsg, shmMsg);
}


TEST(ConversionTraitsTest, JointState)
{
    constexpr int len = 3;
    using JointStates = std::array<any_measurements::JointState, len>;

    JointStates shmMsg;

    sensor_msgs::JointState rosMsg;

    any_measurements::Time myTime;
    myTime.setNow();

    for(unsigned int i=0; i<shmMsg.size(); ++i) {
        shmMsg[i].time_ = myTime;
        shmMsg[i].position_ = static_cast<double>(i);
        shmMsg[i].velocity_ = static_cast<double>(i + shmMsg.size());
        shmMsg[i].effort_ = static_cast<double>(i + 2*shmMsg.size());
    }

    JointStates shmMsgCopy(shmMsg);

    doConversion(shmMsg, rosMsg);

    for(unsigned int i=0; i<shmMsg.size(); i++) {
        EXPECT_EQ(shmMsg[i].time_, shmMsgCopy[i].time_);
        EXPECT_EQ(shmMsg[i].position_, shmMsgCopy[i].position_);
        EXPECT_EQ(shmMsg[i].velocity_, shmMsgCopy[i].velocity_);
        EXPECT_EQ(shmMsg[i].effort_, shmMsgCopy[i].effort_);
    }
}

TEST(ConversionTraitsTest, ExtendedJointState)
{
    constexpr int len = 3;
    using ExtendedJointStates = std::array<any_measurements::ExtendedJointState, len>;

    ExtendedJointStates shmMsg;

    any_msgs::ExtendedJointState rosMsg;

    any_measurements::Time myTime;
    myTime.setNow();

    for(unsigned int i=0; i<shmMsg.size(); ++i) {
        shmMsg[i].time_ = myTime;
        shmMsg[i].position_ = static_cast<double>(i);
        shmMsg[i].velocity_ = static_cast<double>(i + shmMsg.size());
        shmMsg[i].acceleration_ = static_cast<double>(i + 2*shmMsg.size());
        shmMsg[i].effort_ = static_cast<double>(i + 3*shmMsg.size());
    }

    ExtendedJointStates shmMsgCopy(shmMsg);

    doConversion(shmMsg, rosMsg);

    for(unsigned int i=0; i<shmMsg.size(); i++) {
        EXPECT_EQ(shmMsg[i].time_, shmMsgCopy[i].time_);
        EXPECT_EQ(shmMsg[i].position_, shmMsgCopy[i].position_);
        EXPECT_EQ(shmMsg[i].velocity_, shmMsgCopy[i].velocity_);
        EXPECT_EQ(shmMsg[i].acceleration_, shmMsgCopy[i].acceleration_);
        EXPECT_EQ(shmMsg[i].effort_, shmMsgCopy[i].effort_);
    }
}

TEST(ConversionTraitsTest, PointContact)
{
    any_measurements::PointContact shmMsg;
    any_msgs::PointContact rosMsg;
    any_measurements::Time myTime;

    myTime.setNow();
    shmMsg.time_ = myTime;
    shmMsg.wrench_.wrench_.setVector(Eigen::Matrix<double, 6, 1>::Random());
    shmMsg.position_.toImplementation().setRandom();
    shmMsg.twist_.getTranslationalVelocity().setRandom();
    shmMsg.twist_.getRotationalVelocity().setRandom();
    shmMsg.normal_.toImplementation().setRandom();
    shmMsg.state_ = 1;

    any_measurements::PointContact shmMsgCopy(shmMsg);

    doConversion(shmMsg, rosMsg);

    EXPECT_EQ(shmMsg.time_, shmMsgCopy.time_);
    EXPECT_EQ(shmMsg.wrench_.wrench_, shmMsgCopy.wrench_.wrench_);
    EXPECT_EQ(shmMsg.position_, shmMsgCopy.position_);
    EXPECT_EQ(shmMsg.twist_.getTranslationalVelocity(), shmMsgCopy.twist_.getTranslationalVelocity());
    EXPECT_EQ(shmMsg.twist_.getRotationalVelocity(), shmMsgCopy.twist_.getRotationalVelocity());
    EXPECT_EQ(shmMsg.normal_, shmMsgCopy.normal_);
    EXPECT_EQ(shmMsg.state_, shmMsgCopy.state_);
}