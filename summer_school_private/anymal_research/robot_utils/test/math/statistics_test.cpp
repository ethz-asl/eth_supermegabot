/*!
* @file     statistics_test.cpp
* @author   Gabriel Hottiger
* @date     Dec 19, 2017
* @brief
*/

#include <gtest/gtest.h>
#include <numeric>
#include "robot_utils/math/statistics.hpp"

typedef ::testing::Types<
  double,
  float,
  int
> StatisticsTestTypes;

template <typename T>
struct StatisticsTest : public ::testing::Test {
  using Type = T;

  StatisticsTest() : v_(1000) {
    std::srand(std::time(nullptr));
    std::generate(v_.begin(), v_.end(), [](){return robot_utils::generateRandomNumber<Type>();});
  }

  std::vector<Type> v_;
};

TYPED_TEST_CASE(StatisticsTest, StatisticsTestTypes);


TYPED_TEST(StatisticsTest, elementWiseAbs)
{

  auto vAbs = robot_utils::abs(TestFixture::v_);
  for(int i = 0; i < TestFixture::v_.size(); ++i) {
    ASSERT_EQ(std::abs(TestFixture::v_[i]), vAbs[i]);
  }

  vAbs = TestFixture::v_;
  robot_utils::applyAbs(vAbs);

  for(int i = 0; i < TestFixture::v_.size(); ++i) {
    ASSERT_EQ(std::abs(TestFixture::v_[i]), vAbs[i]);
  }
}

TYPED_TEST(StatisticsTest, meanTest)
{
  auto mean = robot_utils::mean(TestFixture::v_);
  auto estMean = 0.0;
  for(const auto& val : TestFixture::v_) {
    estMean += val;
  }

  estMean /= TestFixture::v_.size();
  ASSERT_EQ(estMean, mean);
}

TYPED_TEST(StatisticsTest, varTest)
{
  auto mean = robot_utils::mean(TestFixture::v_);
  auto var = robot_utils::var(TestFixture::v_);
  auto estVar = 0.0;
  for(const auto& val : TestFixture::v_) {
    estVar += (val-mean)*(val-mean);
  }
  estVar /= (TestFixture::v_.size()-1);

  ASSERT_EQ(estVar, var);
}




