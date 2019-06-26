/*!
* @file     EnumArrayTest.hpp
* @author   Gabriel Hottiger
* @date     Oct 30, 2017
* @brief
*/

#include <gtest/gtest.h>

#include <std_utils/std_utils.hpp>

CONSECUTIVE_ENUM(TestEnum, E_0, E_1, E_2, E_3);

TEST(KeyArray, insert) {
  std_utils::KeyArray<TestEnum> keyContainer = {{ std_utils::make_key(TestEnum::E_0, "E_0", 5),
                                                  std_utils::make_key(TestEnum::E_1, "E_1"),      // defaults to 1 static_cast to underlying type
                                                  std_utils::make_key(TestEnum::E_2, "E_2", 12),
                                                  std_utils::make_key(TestEnum::E_3, "E_3") }};   // defaults to 3 static_cast to underlying type

  EXPECT_EQ(5, keyContainer.atId(5).getId());
  EXPECT_EQ(strcmp("E_0", keyContainer.atId(5).getName()), 0);
  EXPECT_EQ(TestEnum::E_0, keyContainer.atId(5).getEnum());

  EXPECT_EQ(1, keyContainer.at(TestEnum::E_1).getId());
  EXPECT_EQ(strcmp("E_1", keyContainer.at(TestEnum::E_1).getName()), 0);

  EXPECT_EQ(12, keyContainer.get<TestEnum::E_2>().getId());

  EXPECT_EQ(TestEnum::E_2, keyContainer.atId(12).getEnum());
  EXPECT_EQ(5, keyContainer.atName("E_0").getId());

  EXPECT_EQ(4, keyContainer.size());
}


