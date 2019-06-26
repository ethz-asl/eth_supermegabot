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
CONSECUTIVE_ENUM(EnumA1, A_0, A_1, A_2)
CONSECUTIVE_ENUM(EnumA2, A_3, A_4)
CONSECUTIVE_ENUM(EnumB,  B_0, B_1, B_2, B_3, B_4)
CONSECUTIVE_ENUM(EnumB1, B_0, B_1, B_2)
CONSECUTIVE_ENUM(EnumB2, B_3, B_4)

//! Define KV-pair helpers
template <EnumA enumA, EnumB enumB>
using kvAB = std_utils::KeyValuePair<EnumA, EnumB, enumA, enumB>;

template <EnumA enumA, unsigned int value>
using kvAUint = std_utils::KeyValuePair<EnumA, unsigned int, enumA, value>;

template <EnumA1 enumA1, EnumA enumA>
using kvA1A = std_utils::KeyValuePair<EnumA1, EnumA, enumA1, enumA>;

template <EnumB1 enumB1, EnumB enumB>
using kvB1B = std_utils::KeyValuePair<EnumB1, EnumB, enumB1, enumB>;

template <EnumA1 enumA1, EnumB1 enumB1>
using kvA1B1 = std_utils::KeyValuePair<EnumA1, EnumB1, enumA1, enumB1>;

template <EnumA2 enumA2, EnumA enumA>
using kvA2A = std_utils::KeyValuePair<EnumA2, EnumA, enumA2, enumA>;

template <EnumB2 enumB2, EnumB enumB>
using kvB2B = std_utils::KeyValuePair<EnumB2, EnumB, enumB2, enumB>;

template <EnumA2 enumA2, EnumB2 enumB2>
using kvA2B2 = std_utils::KeyValuePair<EnumA2, EnumB2, enumA2, enumB2>;

//! Define some mappings
using mapAB1 = std_utils::CompileTimeMap<EnumA, EnumB,
                                         kvAB<EnumA::A_0, EnumB::B_0>,
                                         kvAB<EnumA::A_1, EnumB::B_1>,
                                         kvAB<EnumA::A_2, EnumB::B_2>>;
using mapAB2 = std_utils::CompileTimeMap<EnumA, EnumB,
                                         kvAB<EnumA::A_3, EnumB::B_3>,
                                         kvAB<EnumA::A_4, EnumB::B_4>>;

using mapAB = std_utils::CompileTimeMap<EnumA, EnumB,
                                        kvAB<EnumA::A_0, EnumB::B_0>,
                                        kvAB<EnumA::A_1, EnumB::B_1>,
                                        kvAB<EnumA::A_2, EnumB::B_2>,
                                        kvAB<EnumA::A_3, EnumB::B_3>,
                                        kvAB<EnumA::A_4, EnumB::B_4>>;

using mapABDuplicates = std_utils::CompileTimeMap<EnumA, EnumB,
                                                  kvAB<EnumA::A_0, EnumB::B_0>,
                                                  kvAB<EnumA::A_1, EnumB::B_1>,
                                                  kvAB<EnumA::A_2, EnumB::B_0>,
                                                  kvAB<EnumA::A_3, EnumB::B_3>,
                                                  kvAB<EnumA::A_4, EnumB::B_0>>;

using mapAUint = std_utils::CompileTimeMap<EnumA, unsigned int,
                                           kvAUint<EnumA::A_0, 2u>,
                                           kvAUint<EnumA::A_1, 6u>,
                                           kvAUint<EnumA::A_2, 1u>>;

using mapA1A = std_utils::CompileTimeMap<EnumA1, EnumA,
                                         kvA1A<EnumA1::A_0, EnumA::A_0>,
                                         kvA1A<EnumA1::A_1, EnumA::A_1>,
                                         kvA1A<EnumA1::A_2, EnumA::A_2>>;

using mapB1B = std_utils::CompileTimeMap<EnumB1, EnumB,
                                         kvB1B<EnumB1::B_0, EnumB::B_0>,
                                         kvB1B<EnumB1::B_1, EnumB::B_1>,
                                         kvB1B<EnumB1::B_2, EnumB::B_2>>;

using mapA1B1 = std_utils::CompileTimeMap<EnumA1, EnumB1,
                                          kvA1B1<EnumA1::A_0, EnumB1::B_0>,
                                          kvA1B1<EnumA1::A_1, EnumB1::B_1>,
                                          kvA1B1<EnumA1::A_2, EnumB1::B_2>>;

using mapA2A = std_utils::CompileTimeMap<EnumA2, EnumA,
                                         kvA2A<EnumA2::A_3, EnumA::A_3>,
                                         kvA2A<EnumA2::A_4, EnumA::A_4>>;

using mapB2B = std_utils::CompileTimeMap<EnumB2, EnumB,
                                         kvB2B<EnumB2::B_3, EnumB::B_3>,
                                         kvB2B<EnumB2::B_4, EnumB::B_4>>;

using mapA2B2 = std_utils::CompileTimeMap<EnumA2, EnumB2,
                                          kvA2B2<EnumA2::A_3, EnumB2::B_3>,
                                          kvA2B2<EnumA2::A_4, EnumB2::B_4>>;

template<EnumA enumA, EnumB enumB>
struct Functor {
  bool operator()(int& counter) { return ++counter < 4; }
};

template<EnumA enumA, EnumB enumB>
struct FunctorRange {
  bool operator()(int& counter, int increment) { counter += increment; return enumA != EnumA::A_1; }
};

template<typename T, T Val>
struct TestCompileTime : std::true_type { };


TEST(CompileTimeMap, findAll) {
  using cts_duplicates = mapABDuplicates::findAll<EnumB::B_0>;
  using cts_expected = std_utils::CompileTimeSet<EnumA, EnumA::A_0, EnumA::A_2, EnumA::A_4>;
  constexpr bool findAllSuccessful = std::is_same<cts_duplicates, cts_expected>::value;
  EXPECT_TRUE(findAllSuccessful);
}

TEST(CompileTimeMap, findAllRuntime) {
  auto duplicatesVector = mapABDuplicates::findVector(EnumB::B_0);

  EXPECT_EQ(EnumA::A_0, duplicatesVector.at(0));
  EXPECT_EQ(EnumA::A_2, duplicatesVector.at(1));
  EXPECT_EQ(EnumA::A_4, duplicatesVector.at(2));

  EXPECT_EQ(3, duplicatesVector.size());
}

TEST(CompileTimeMap, forEachFunctor) {
  // Iteration is stopped at 4
  int counter = 0;
  EXPECT_FALSE(mapAB::forEach<Functor>(counter));
  EXPECT_EQ(counter, 4);

  // Iteration is not stopped, size of map smaller than 4
  counter = 0;
  EXPECT_TRUE(mapAB1::forEach<Functor>(counter));
  EXPECT_EQ(counter, 3);
}

TEST(CompileTimeMap, forEachStdFunction) {
  int counter = 0;
  auto f = [&counter](const EnumA& enumA, const EnumB& enumB){ return ++counter < 4; };

  // Iteration is stopped at 4
  EXPECT_FALSE(mapAB::forEach(f));
  EXPECT_EQ(counter, 4);

  // Iteration is not stopped, size of map smaller than 4
  counter = 0;
  EXPECT_TRUE(mapAB1::forEach(f));
  EXPECT_EQ(counter, 3);
}

TEST(CompileTimeMap, forEachRangeFunctor) {
  constexpr int increment = 2;
  int counter = 0;

  // Iteration is directly stopped
  EXPECT_FALSE(mapAB::forEachRange<FunctorRange>(EnumA::A_1, EnumA::A_4, counter, increment));
  EXPECT_EQ(increment, counter);

  // Iteration stopped at A_1
  counter = 0;
  EXPECT_FALSE(mapAB::forEachRange<FunctorRange>(EnumA::A_0, EnumA::A_3, counter, increment));
  EXPECT_EQ(2*increment, counter);

  // Iteration runs smoothly
  counter = 0;
  EXPECT_TRUE(mapAB::forEachRange<FunctorRange>(EnumA::A_2, EnumA::A_3, counter, increment));
  EXPECT_EQ(2*increment, counter);

  // Invalid range
  counter = 0;
  EXPECT_FALSE(mapAB::forEachRange<FunctorRange>(EnumA::A_3, EnumA::A_2, counter, increment));
  EXPECT_EQ(0, counter);
}

TEST(CompileTimeMap, forEachRangeStdFunction) {
  constexpr int increment = 2;
  int counter = 0;
  auto f = [&counter, increment](const EnumA& enumA, const EnumB& enumB){ counter += increment; return enumA != EnumA::A_1; };

  // Iteration is directly stopped
  EXPECT_FALSE(mapAB::forEachRange(EnumA::A_1, EnumA::A_4, f));
  EXPECT_EQ(increment, counter);

  // Iteration stopped at A_1
  counter = 0;
  EXPECT_FALSE(mapAB::forEachRange(EnumA::A_0, EnumA::A_3, f));
  EXPECT_EQ(2*increment, counter);

  // Iteration runs smoothly
  counter = 0;
  EXPECT_TRUE(mapAB::forEachRange(EnumA::A_2, EnumA::A_3, f));
  EXPECT_EQ(2*increment, counter);

  // Invalid range
  counter = 0;
  EXPECT_FALSE(mapAB::forEachRange(EnumA::A_3, EnumA::A_2, f));
  EXPECT_EQ(0, counter);
}

TEST(CompileTimeMap, at) {
  constexpr bool isCompileTime = TestCompileTime<EnumB, mapAB::at(EnumA::A_0)>::value;
  EXPECT_TRUE(isCompileTime);
  EXPECT_EQ(mapAB::at(EnumA::A_0), EnumB::B_0);
  EXPECT_EQ(mapAB::at(EnumA::A_1), EnumB::B_1);
  EXPECT_EQ(mapAB::at(EnumA::A_2), EnumB::B_2);
  EXPECT_EQ(mapAB::at(EnumA::A_3), EnumB::B_3);
  EXPECT_EQ(mapAB::at(EnumA::A_4), EnumB::B_4);
  EXPECT_THROW(mapAB::at(EnumA::SIZE), std::out_of_range);
}

TEST(CompileTimeMap, find) {
  constexpr bool isCompileTime = TestCompileTime<EnumA, mapAB::find(EnumB::B_0)>::value;
  EXPECT_TRUE(isCompileTime);
  EXPECT_EQ(mapAB::find(EnumB::B_0), EnumA::A_0);
  EXPECT_EQ(mapAB::find(EnumB::B_1), EnumA::A_1);
  EXPECT_EQ(mapAB::find(EnumB::B_2), EnumA::A_2);
  EXPECT_EQ(mapAB::find(EnumB::B_3), EnumA::A_3);
  EXPECT_EQ(mapAB::find(EnumB::B_4), EnumA::A_4);
  EXPECT_THROW(mapAB::find(EnumB::SIZE), std::out_of_range);
}

TEST(CompileTimeMap, contains) {
  constexpr bool isCompileTime = TestCompileTime<bool, mapAB::contains(EnumA::A_0)>::value;
  EXPECT_TRUE(isCompileTime);
  EXPECT_TRUE(mapAB::contains(EnumA::A_0));
  EXPECT_FALSE(mapAB::contains(EnumA::SIZE));
}

TEST(CompileTimeMap, containsRange) {
  constexpr bool isCompileTime = TestCompileTime<bool, mapAB::containsRange(EnumA::A_1, EnumA::A_4)>::value;
  EXPECT_TRUE(isCompileTime);
  EXPECT_TRUE(mapAB::containsRange(EnumA::A_1, EnumA::A_4));
  EXPECT_TRUE(mapAB::containsRange(EnumA::A_0, EnumA::A_3));
  EXPECT_FALSE(mapAB::containsRange(EnumA::A_2, EnumA::A_1));
  EXPECT_FALSE(mapAB::containsRange(EnumA::A_2, EnumA::SIZE));
  EXPECT_FALSE(mapAB::containsRange(EnumA::SIZE, EnumA::A_1));
}

TEST(CompileTimeMap, containsValue) {
  constexpr bool isCompileTime = TestCompileTime<bool, mapAB::containsValue(EnumB::B_0)>::value;
  EXPECT_TRUE(isCompileTime);
  EXPECT_TRUE(mapAB::containsValue(EnumB::B_0));
  EXPECT_FALSE(mapAB::containsValue(EnumB::SIZE));
}

TEST(CompileTimeMap, allValuesUnique) {
  constexpr bool isCompileTime = TestCompileTime<bool, mapAB::allValuesUnique()>::value;
  EXPECT_TRUE(isCompileTime);
  EXPECT_TRUE(mapAB::allValuesUnique());
  EXPECT_FALSE(mapABDuplicates::allValuesUnique());
}

TEST(CompileTimeMap, size) {
  constexpr bool isCompileTime = TestCompileTime<std::size_t, mapAB::size()>::value;
  EXPECT_TRUE(isCompileTime);
  EXPECT_EQ(5, mapAB::size());
  EXPECT_EQ(3, mapAB1::size());
  EXPECT_EQ(2, mapAB2::size());
}

TEST(CompileTimeMap, concatenate) {
  // Concatenate in right order -> types should match
  using mapABConcatenated = std_utils::ctm_concatenate_t<EnumA, EnumB, mapAB1, mapAB2>;
  constexpr bool mapABConcatenatedSuccessful = std::is_same<mapABConcatenated, mapAB>::value;
  EXPECT_TRUE( mapABConcatenatedSuccessful );

  // Concatenate in wrong order -> types shouldn't match
  using mapABConcatenatedWrongOrder = std_utils::ctm_concatenate_t<EnumA, EnumB, mapAB2, mapAB1>;
  constexpr bool mapABConcatenatedWrongOrderSuccessful = std::is_same<mapABConcatenatedWrongOrder, mapAB>::value;
  EXPECT_FALSE( mapABConcatenatedWrongOrderSuccessful );

  // Contents should match anyway
  for(int i = 0; i < static_cast<int>(EnumA::SIZE); ++i) {
    EXPECT_EQ(mapABConcatenated::at(static_cast<EnumA>(i)), mapAB::at(static_cast<EnumA>(i)));
    EXPECT_EQ(mapABConcatenatedWrongOrder::at(static_cast<EnumA>(i)), mapAB::at(static_cast<EnumA>(i)));
  }
}

TEST(CompileTimeMap, insert_back) {
  using mapAB1_ib = std_utils::ctm_insert_back_t<EnumA, EnumB, mapAB1, kvAB<EnumA::A_3, EnumB::B_3>>;
  using mapAB_ib = std_utils::ctm_insert_back_t<EnumA, EnumB, mapAB1_ib, kvAB<EnumA::A_4, EnumB::B_4>>;

  constexpr bool mapABBackInsertionSuccessful = std::is_same<mapAB_ib, mapAB>::value;
  EXPECT_TRUE( mapABBackInsertionSuccessful );
}

TEST(CompileTimeMap, insert_front) {
  using mapAB2_if = std_utils::ctm_insert_front_t<EnumA, EnumB, mapAB2, kvAB<EnumA::A_2, EnumB::B_2>>;
  using mapAB2_if2 = std_utils::ctm_insert_front_t<EnumA, EnumB, mapAB2_if, kvAB<EnumA::A_1, EnumB::B_1>>;
  using mapAB_if = std_utils::ctm_insert_front_t<EnumA, EnumB, mapAB2_if2, kvAB<EnumA::A_0, EnumB::B_0>>;

  constexpr bool mapABFrontInsertionSuccessful = std::is_same<mapAB_if, mapAB>::value;
  EXPECT_TRUE( mapABFrontInsertionSuccessful );
}

TEST(CompileTimeMap, erase) {
  using mapAB_er = std_utils::ctm_erase_t<EnumA, EnumB, mapAB, EnumA::A_3>;
  using mapAB1_er = std_utils::ctm_erase_t<EnumA, EnumB, mapAB_er, EnumA::A_4>;
  constexpr bool mapABEraseSuccessful = std::is_same<mapAB1_er, mapAB1>::value;
  EXPECT_TRUE( mapABEraseSuccessful );
}

TEST(CompileTimeMap, add_to_values) {
  using mapAUintPlus2 = std_utils::ctm_add_to_values_t<EnumA, unsigned int, mapAUint, 2>;
  EXPECT_EQ(mapAUintPlus2::at(EnumA::A_0), 4);
  EXPECT_EQ(mapAUintPlus2::at(EnumA::A_1), 8);
  EXPECT_EQ(mapAUintPlus2::at(EnumA::A_2), 3);
}

TEST(CompileTimeMap, from_sets) {
  using keySet = std_utils::CompileTimeSet<EnumA, EnumA::A_0, EnumA::A_1, EnumA::A_2>;
  using valueSet = std_utils::CompileTimeSet<EnumB, EnumB::B_0, EnumB::B_1, EnumB::B_2>;
  using mapAB1fromSets = std_utils::ctm_from_sets_t<EnumA, EnumB, keySet, valueSet>;
  constexpr bool mapAB1FromSetsSuccessful = std::is_same<mapAB1fromSets, mapAB1>::value;
  EXPECT_TRUE( mapAB1FromSetsSuccessful );
}

TEST(CompileTimeMap, transform) {
  using mapABTransformed = std_utils::ctm_transform_t<EnumA, EnumB, mapA1B1, mapA1A, mapB1B>;
  constexpr bool mapABFullTransformSuccessful = std::is_same<mapABTransformed, mapAB1>::value;
  EXPECT_TRUE( mapABFullTransformSuccessful );

  using mapA1BTransformed = std_utils::ctm_transform_t<EnumA1, EnumB, mapA1B1, void, mapB1B>;
  EXPECT_EQ(3, mapA1BTransformed::size());
  EXPECT_EQ(EnumB::B_0, mapA1BTransformed::at(EnumA1::A_0));
  EXPECT_EQ(EnumB::B_1, mapA1BTransformed::at(EnumA1::A_1));
  EXPECT_EQ(EnumB::B_2, mapA1BTransformed::at(EnumA1::A_2));

  using mapAB1Transformed = std_utils::ctm_transform_t<EnumA, EnumB1, mapA1B1, mapA1A, void>;
  EXPECT_EQ(3, mapAB1Transformed::size());
  EXPECT_EQ(EnumB1::B_0, mapAB1Transformed::at(EnumA::A_0));
  EXPECT_EQ(EnumB1::B_1, mapAB1Transformed::at(EnumA::A_1));
  EXPECT_EQ(EnumB1::B_2, mapAB1Transformed::at(EnumA::A_2));
}

TEST(CompileTimeMap, combine) {
  using mapABCombined = std_utils::ctm_combine_t<EnumA, EnumB, mapA1B1, mapA2B2, mapA1A, mapA2A, mapB1B, mapB2B>;
  constexpr bool mapABCombinedSuccessful = std::is_same<mapABCombined, mapAB>::value;
  EXPECT_TRUE( mapABCombinedSuccessful );
}

TEST(CompileTimeMap, from_subenum) {
  using mapA1AFromSubenum = std_utils::ctm_from_subenum_t<EnumA, EnumA1, 0, 0>;
  using mapA2AFromSubenum = std_utils::ctm_from_subenum_t<EnumA, EnumA2, static_cast<int>(EnumA1::SIZE), 0>;
  constexpr bool mapA1AFromSubenumSuccessful = std::is_same<mapA1AFromSubenum, mapA1A>::value;
  constexpr bool mapA2AFromSubenumSuccessful = std::is_same<mapA2AFromSubenum, mapA2A>::value;
  EXPECT_TRUE( mapA1AFromSubenumSuccessful );
  EXPECT_TRUE( mapA2AFromSubenumSuccessful );
}

TEST(CompileTimeMap, invert) {
  using mapBA = std_utils::ctm_invert_t<EnumB, EnumA, mapAB>;
  EXPECT_EQ(mapBA::at(EnumB::B_0), EnumA::A_0);
  EXPECT_EQ(mapBA::at(EnumB::B_1), EnumA::A_1);
  EXPECT_EQ(mapBA::at(EnumB::B_2), EnumA::A_2);
  EXPECT_EQ(mapBA::at(EnumB::B_3), EnumA::A_3);
  EXPECT_EQ(mapBA::at(EnumB::B_4), EnumA::A_4);
}

