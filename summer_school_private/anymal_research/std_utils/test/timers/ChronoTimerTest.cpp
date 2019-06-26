/*!
* @file     ChronoTimerTest.cpp
* @author   Dario Bellicoso
* @date     February 14, 2016
* @brief
*/
#include <gtest/gtest.h>

#include <std_utils/timers/ChronoTimer.hpp>

TEST(Timers, GetAverageTimeMsec) {
  using namespace std_utils;

  SteadyClockTimer steadyClockTimer;
  int sleepTimeMsec = 15;

  const int rounds = 100;
  for (int k=0; k<rounds; k++) {
    steadyClockTimer.pinTime();
    usleep(sleepTimeMsec*1000.0);
    steadyClockTimer.splitTime();
  }

  EXPECT_NEAR(sleepTimeMsec, steadyClockTimer.getAverageElapsedTimeMSec(), 1e-0);

  EXPECT_DOUBLE_EQ(steadyClockTimer.getAverageElapsedTimeMSec()/1000.0, steadyClockTimer.getAverageElapsedTimeSec());
  EXPECT_DOUBLE_EQ(steadyClockTimer.getAverageElapsedTimeMSec()*1000.0, steadyClockTimer.getAverageElapsedTimeUSec());
  EXPECT_DOUBLE_EQ(steadyClockTimer.getAverageElapsedTimeMSec()*1000.0*1000.0, steadyClockTimer.getAverageElapsedTimeNSec());

}
