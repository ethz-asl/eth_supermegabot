/*!
 * @file     CompileTimeMapTest.cpp
 * @author   Gabriel Hottiger
 * @date     Apr, 2018
 * @brief    Test the compile time map class
 */

#include <gtest/gtest.h>

#include <std_utils/std_utils.hpp>


/*!
 * @file     CompileTimeMapTest.cpp
 * @author   Gabriel Hottiger
 * @date     Apr, 2018
 * @brief    Test the compile time map class
 */

#include <gtest/gtest.h>

#include <std_utils/std_utils.hpp>

//! Define some helper enums for testing
CONSECUTIVE_ENUM(EnumA,  A_0, A_1, A_2, A_3, A_4)

//! Define some sets
using subSetA1 = std_utils::CompileTimeSet<EnumA, EnumA::A_0, EnumA::A_1, EnumA::A_2>;
using subSetA2 = std_utils::CompileTimeSet<EnumA, EnumA::A_3, EnumA::A_4>;
using setA = std_utils::CompileTimeSet<EnumA, EnumA::A_0, EnumA::A_1, EnumA::A_2, EnumA::A_3, EnumA::A_4>;

template<EnumA enumA>
struct Functor {
  bool operator()(int& counter) { return ++counter < 4; }
};

template<EnumA enumA>
struct FunctorRange {
  bool operator()(int& counter, int increment) { counter += increment; return enumA != EnumA::A_1; }
};

template<typename T, T Val>
struct TestCompileTime : std::true_type { };

TEST(CompileTimeSet, forEachFunctor) {
  // Iteration is stopped at 4
  int counter = 0;
  EXPECT_FALSE(setA::forEach<Functor>(counter));
  EXPECT_EQ(counter, 4);

  // Iteration is not stopped, size of map smaller than 4
  counter = 0;
  EXPECT_TRUE(subSetA1::forEach<Functor>(counter));
  EXPECT_EQ(counter, 3);
}

TEST(CompileTimeSet, forEachStdFunction) {
  int counter = 0;
  auto f = [&counter](const EnumA& enumA){ return ++counter < 4; };

  // Iteration is stopped at 4
  EXPECT_FALSE(setA::forEach(f));
  EXPECT_EQ(counter, 4);

  // Iteration is not stopped, size of set smaller than 4
  counter = 0;
  EXPECT_TRUE(subSetA1::forEach(f));
  EXPECT_EQ(counter, 3);
}

TEST(CompileTimeSet, forEachRangeFunctor) {
  constexpr int increment = 2;
  int counter = 0;

  // Iteration is directly stopped
  EXPECT_FALSE(setA::forEachRange<FunctorRange>(EnumA::A_1, EnumA::A_4, counter, increment));
  EXPECT_EQ(increment, counter);

  // Iteration stopped at A_1
  counter = 0;
  EXPECT_FALSE(setA::forEachRange<FunctorRange>(EnumA::A_0, EnumA::A_3, counter, increment));
  EXPECT_EQ(2*increment, counter);

  // Iteration runs smoothly
  counter = 0;
  EXPECT_TRUE(setA::forEachRange<FunctorRange>(EnumA::A_2, EnumA::A_3, counter, increment));
  EXPECT_EQ(2*increment, counter);

  // Invalid range
  counter = 0;
  EXPECT_FALSE(setA::forEachRange<FunctorRange>(EnumA::A_3, EnumA::A_2, counter, increment));
  EXPECT_EQ(0, counter);
}

TEST(CompileTimeSet, forEachRangeStdFunction) {
  constexpr int increment = 2;
  int counter = 0;
  auto f = [&counter, increment](const EnumA& enumA){ counter += increment; return enumA != EnumA::A_1; };

  // Iteration is directly stopped
  EXPECT_FALSE(setA::forEachRange(EnumA::A_1, EnumA::A_4, f));
  EXPECT_EQ(increment, counter);

  // Iteration stopped at A_1
  counter = 0;
  EXPECT_FALSE(setA::forEachRange(EnumA::A_0, EnumA::A_3, f));
  EXPECT_EQ(2*increment, counter);

  // Iteration runs smoothly
  counter = 0;
  EXPECT_TRUE(setA::forEachRange(EnumA::A_2, EnumA::A_3, f));
  EXPECT_EQ(2*increment, counter);

  // Invalid range
  counter = 0;
  EXPECT_FALSE(setA::forEachRange(EnumA::A_3, EnumA::A_2, f));
  EXPECT_EQ(0, counter);
}

TEST(CompileTimeSet, contains) {
  EXPECT_TRUE(setA::contains(EnumA::A_0));
  EXPECT_TRUE(setA::contains(EnumA::A_1));
  EXPECT_TRUE(setA::contains(EnumA::A_2));
  EXPECT_TRUE(setA::contains(EnumA::A_3));
  EXPECT_TRUE(setA::contains(EnumA::A_4));
  EXPECT_FALSE(setA::contains(EnumA::SIZE));
}

TEST(CompileTimeSet, containsRange) {
  EXPECT_TRUE(setA::containsRange(EnumA::A_1, EnumA::A_4));
  EXPECT_TRUE(setA::containsRange(EnumA::A_0, EnumA::A_3));
  EXPECT_FALSE(setA::containsRange(EnumA::A_2, EnumA::A_1));
  EXPECT_FALSE(setA::containsRange(EnumA::A_2, EnumA::SIZE));
  EXPECT_FALSE(setA::containsRange(EnumA::SIZE, EnumA::A_1));
}

TEST(CompileTimeSet, size) {
  EXPECT_EQ(5, setA::size());
  EXPECT_EQ(3, subSetA1::size());
  EXPECT_EQ(2, subSetA2::size());
}

TEST(CompileTimeSet, concatenate) {
  // Concatenate in right order -> types should match
  using setAConcatenated = std_utils::cts_concatenate_t<EnumA, subSetA1, subSetA2>;
  constexpr bool setAConcatenatedSuccessful = std::is_same<setAConcatenated, setA>::value;
  EXPECT_TRUE( setAConcatenatedSuccessful );

  // Concatenate in wrong order -> types shouldn't match
  using setAConcatenatedWrongOrder = std_utils::cts_concatenate_t<EnumA, subSetA2, subSetA1>;
  constexpr bool setAConcatenatedWrongOrderSuccessful = std::is_same<setAConcatenatedWrongOrder, setA>::value;
  EXPECT_FALSE( setAConcatenatedWrongOrderSuccessful );

  // Contents should match anyway
  for(int i = 0; i < static_cast<int>(EnumA::SIZE); ++i) {
    EXPECT_TRUE(setAConcatenated::contains(static_cast<EnumA>(i)));
    EXPECT_TRUE(setAConcatenatedWrongOrder::contains(static_cast<EnumA>(i)));
  }
}

TEST(CompileTimeSet, insert_back) {
  using setA1_ib = std_utils::cts_insert_back_t<EnumA, subSetA1, EnumA::A_3>;
  using setA_ib = std_utils::cts_insert_back_t<EnumA, setA1_ib, EnumA::A_4>;

  constexpr bool setABackInsertionSuccessful = std::is_same<setA_ib, setA>::value;
  EXPECT_TRUE( setABackInsertionSuccessful );
}

TEST(CompileTimeSet, insert_front) {
  using setA1_if = std_utils::cts_insert_front_t<EnumA, subSetA2, EnumA::A_2>;
  using setA2_if = std_utils::cts_insert_front_t<EnumA, setA1_if, EnumA::A_1>;
  using setA_if = std_utils::cts_insert_front_t<EnumA, setA2_if, EnumA::A_0>;

  constexpr bool setAFrontInsertionSuccessful = std::is_same<setA_if, setA>::value;
  EXPECT_TRUE( setAFrontInsertionSuccessful );
}

TEST(CompileTimeSet, insert_front_if) {
  using setA1_if = std_utils::cts_insert_front_if_t<EnumA, subSetA2, EnumA::A_2, true>;
  using setA2_if = std_utils::cts_insert_front_if_t<EnumA, setA1_if, EnumA::A_1, true>;
  using setA3_if = std_utils::cts_insert_front_if_t<EnumA, setA2_if, EnumA::A_1, false>;
  using setA_if = std_utils::cts_insert_front_if_t<EnumA, setA3_if, EnumA::A_0, true>;

  constexpr bool setAConditionalFrontInsertionSuccessful = std::is_same<setA_if, setA>::value;
  EXPECT_TRUE( setAConditionalFrontInsertionSuccessful );
}

TEST(CompileTimeSet, erase) {
  using setA_er = std_utils::cts_erase_t<EnumA, setA,  EnumA::A_3>;
  using setA_er1 = std_utils::cts_erase_t<EnumA, setA_er,  EnumA::A_4>;
  constexpr bool setAEraseSuccessful = std::is_same<setA_er1, subSetA1>::value;
  EXPECT_TRUE( setAEraseSuccessful );
}

TEST(CompileTimeSet, from_sequence) {
  using setASequence =
    std_utils::cts_from_sequence_t<EnumA, std_utils::make_index_sequence_t<static_cast<int>(EnumA::SIZE)>>;

  constexpr bool setAFromDequenceSuccessful = std::is_same<setASequence, setA>::value;
  EXPECT_TRUE( setAFromDequenceSuccessful );
}

TEST(CompileTimeSet, from_enum) {
  using setAEnum = std_utils::cts_from_enum_t<EnumA>;
  constexpr bool setAFromEnumSuccessful = std::is_same<setAEnum, setA>::value;
  EXPECT_TRUE( setAFromEnumSuccessful );
}