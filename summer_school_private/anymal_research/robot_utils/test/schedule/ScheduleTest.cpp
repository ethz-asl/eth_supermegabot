/*!
* @file     ScheduleTest.cpp
* @author   Christian Gehring
* @date     March 20, 2015
* @brief
*/
#include <gtest/gtest.h>

#include <robot_utils/schedule/Schedule.hpp>
#include <robot_utils/schedule/ProfileStep.hpp>
#include <robot_utils/schedule/ProfileRamp.hpp>
#include <robot_utils/schedule/ProfileLogChirpUpSweep.hpp>


TEST(Schedule, construct) {
  using namespace robot_utils;
  Schedule<double> schedule(100.0);
  schedule.addProfile(std::shared_ptr<Profile<double>>(new ProfileStep<double>(10.0, 1.0)));
  schedule.addProfile(std::shared_ptr<Profile<double>>(new ProfileStep<double>(20.0, 2.0)));
  schedule.addProfile(new ProfileStep<double>(30.0, 3.0));

  // time before start time of schedule
  EXPECT_EQ(10.0, schedule.getValue(-1.0));
  // time after time of schedule
  EXPECT_EQ(30.0, schedule.getValue(200));

  EXPECT_EQ(10.0, schedule.getValue(100.0));
  EXPECT_EQ(10.0, schedule.getValue(100.5));
  EXPECT_EQ(20.0, schedule.getValue(101.0));
  EXPECT_EQ(20.0, schedule.getValue(102.0));
  EXPECT_EQ(30.0, schedule.getValue(103.0));

}

TEST(Schedule, rampProfile) {
  using namespace robot_utils;
  ProfileRamp<double> profile(10.0, 20.0, 5.0);
  EXPECT_EQ(0.0, profile.getStartTime());

  profile.setStartTime(100.0);
  EXPECT_EQ(100.0, profile.getStartTime());
  EXPECT_EQ(5.0, profile.getDuration());
  EXPECT_EQ(105.0, profile.getEndTime());

  // start time
  EXPECT_EQ(10.0, profile.getValue(100.0));
  // end time
  EXPECT_EQ(20.0, profile.getValue(105.0));
  // middle
  EXPECT_EQ(15.0, profile.getValue(102.5));
}

TEST(Schedule, logChirpUpSweepProfile) {
  using namespace robot_utils;
  ProfileLogChirpUpSweep<double> profile(10.0, 20.0, 1.0, 20.0, 5.0);
  EXPECT_EQ(0.0, profile.getStartTime());

  profile.setStartTime(100.0);
  EXPECT_EQ(100.0, profile.getStartTime());
  EXPECT_EQ(5.0, profile.getDuration());
  EXPECT_EQ(105.0, profile.getEndTime());


}
