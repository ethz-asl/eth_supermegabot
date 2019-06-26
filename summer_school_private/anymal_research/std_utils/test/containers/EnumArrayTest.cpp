/*!
* @file     EnumArrayTest.hpp
* @author   Gabriel Hottiger
* @date     Oct 30, 2017
* @brief
*/

#include <gtest/gtest.h>

#include <std_utils/std_utils.hpp>

CONSECUTIVE_ENUM(TestEnum, E_0, E_1, E_2, E_3);

TEST(EnumArray, insert) {
  std_utils::EnumArray<TestEnum, int> container;

  container.at(TestEnum::E_0) = 1;
  container.at(TestEnum::E_1) = 2;
  container[TestEnum::E_3] = 4;

  EXPECT_EQ(1, container[TestEnum::E_0]);
  EXPECT_EQ(1, container.at(TestEnum::E_0));

  EXPECT_EQ(2, container.get<TestEnum::E_1>());

  EXPECT_EQ(4, container[TestEnum::E_3]);
  EXPECT_EQ(4, container.at(TestEnum::E_3));

  EXPECT_EQ(4, container.size());
}


