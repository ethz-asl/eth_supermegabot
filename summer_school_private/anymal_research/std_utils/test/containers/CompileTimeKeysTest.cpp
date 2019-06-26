/*!
 * @file     CompileTimeKeys.cpp
 * @author   Gabriel Hottiger
 * @date     Oct 30, 2017
 * @brief
 */

#include <gtest/gtest.h>

#include <std_utils/std_utils.hpp>

//! Define some enums
CONSECUTIVE_ENUM(EnumA1, E_0);
CONSECUTIVE_ENUM(EnumA2, E_1, E_2, E_3);
CONSECUTIVE_ENUM(EnumA, E_0, E_1, E_2, E_3);

//! Define some mappings
template <EnumA1 enumA1, EnumA enumA>
using kvA1A = std_utils::KeyValuePair<EnumA1, EnumA, enumA1, enumA>;

template <EnumA2 enumA2, EnumA enumA>
using kvA2A = std_utils::KeyValuePair<EnumA2, EnumA, enumA2, enumA>;

using mapEnumA1ToEnumA = std_utils::CompileTimeMap<EnumA1, EnumA, kvA1A<EnumA1::E_0, EnumA::E_0>>;

using mapEnumA2ToEnumA = std_utils::CompileTimeMap<EnumA2, EnumA, kvA2A<EnumA2::E_1, EnumA::E_1>,
                                                   kvA2A<EnumA2::E_2, EnumA::E_2>, kvA2A<EnumA2::E_3, EnumA::E_3>>;

// Define keys
using ctkAFront = std_utils::CompileTimeKeys<EnumA, std_utils::CompileTimeKey<EnumA, EnumA::E_0, ct_string("E_0"), 5>>;

using ctkABack =
    std_utils::CompileTimeKeys<EnumA,
                               std_utils::CompileTimeKey<EnumA, EnumA::E_1, ct_string("E_1")>,  // defaults to id:1
                               std_utils::CompileTimeKey<EnumA, EnumA::E_2, ct_string("E_2"), 12>,
                               std_utils::CompileTimeKey<EnumA, EnumA::E_3, ct_string("E_3")>>;  // defaults to id:3

using ctkA1 = std_utils::CompileTimeKeys<EnumA1, std_utils::CompileTimeKey<EnumA1, EnumA1::E_0, ct_string("E_0"), 5>>;

using ctkA2 =
    std_utils::CompileTimeKeys<EnumA2,
                               std_utils::CompileTimeKey<EnumA2, EnumA2::E_1, ct_string("E_1")>,  // defaults to id:0
                               std_utils::CompileTimeKey<EnumA2, EnumA2::E_2, ct_string("E_2"), 11>,
                               std_utils::CompileTimeKey<EnumA2, EnumA2::E_3, ct_string("E_3")>>;  // defaults to id:2

using ctkA =
    std_utils::CompileTimeKeys<EnumA, std_utils::CompileTimeKey<EnumA, EnumA::E_0, ct_string("E_0"), 5>,
                               std_utils::CompileTimeKey<EnumA, EnumA::E_1, ct_string("E_1")>,  // defaults to id:1
                               std_utils::CompileTimeKey<EnumA, EnumA::E_2, ct_string("E_2"), 12>,
                               std_utils::CompileTimeKey<EnumA, EnumA::E_3, ct_string("E_3")>>;  // defaults to id:3

// Test compile time helper
template <typename T, T Val>
struct TestCompileTime : std::true_type {};

TEST(CompileTimeKeys, mapEnumToName) {
  constexpr bool isCompileTime = std_utils::strings_equal(ctkA::mapEnumToName(EnumA::E_0), "E_0");
  EXPECT_TRUE(isCompileTime);
  EXPECT_EQ(strcmp(ctkA::mapEnumToName(EnumA::E_0), "E_0"), 0);
  EXPECT_EQ(strcmp(ctkA::mapEnumToName(EnumA::E_1), "E_1"), 0);
  EXPECT_EQ(strcmp(ctkA::mapEnumToName(EnumA::E_2), "E_2"), 0);
  EXPECT_EQ(strcmp(ctkA::mapEnumToName(EnumA::E_3), "E_3"), 0);
  EXPECT_THROW(ctkA::mapEnumToName(EnumA::SIZE), std::out_of_range);
}

TEST(CompileTimeKeys, mapEnumToId) {
  constexpr bool isCompileTime = TestCompileTime<int, ctkA::mapEnumToId(EnumA::E_0)>::value;
  EXPECT_TRUE(isCompileTime);
  EXPECT_EQ(ctkA::mapEnumToId(EnumA::E_0), 5);
  EXPECT_EQ(ctkA::mapEnumToId(EnumA::E_1), 1);
  EXPECT_EQ(ctkA::mapEnumToId(EnumA::E_2), 12);
  EXPECT_EQ(ctkA::mapEnumToId(EnumA::E_3), 3);
  EXPECT_THROW(ctkA::mapEnumToId(EnumA::SIZE), std::out_of_range);
}

TEST(CompileTimeKeys, mapIdToEnum) {
  constexpr bool isCompileTime = TestCompileTime<EnumA, ctkA::mapIdToEnum(5)>::value;
  EXPECT_TRUE(isCompileTime);
  EXPECT_EQ(ctkA::mapIdToEnum(5), EnumA::E_0);
  EXPECT_EQ(ctkA::mapIdToEnum(1), EnumA::E_1);
  EXPECT_EQ(ctkA::mapIdToEnum(12), EnumA::E_2);
  EXPECT_EQ(ctkA::mapIdToEnum(3), EnumA::E_3);
  EXPECT_THROW(ctkA::mapIdToEnum(0), std::out_of_range);
}

TEST(CompileTimeKeys, mapIdToName) {
  constexpr bool isCompileTime = std_utils::strings_equal(ctkA::mapIdToName(5), "E_0");
  EXPECT_TRUE(isCompileTime);
  EXPECT_EQ(strcmp(ctkA::mapIdToName(5), "E_0"), 0);
  EXPECT_EQ(strcmp(ctkA::mapIdToName(1), "E_1"), 0);
  EXPECT_EQ(strcmp(ctkA::mapIdToName(12), "E_2"), 0);
  EXPECT_EQ(strcmp(ctkA::mapIdToName(3), "E_3"), 0);
  EXPECT_THROW(ctkA::mapIdToName(0), std::out_of_range);
}

TEST(CompileTimeKeys, mapNameToEnum) {
  constexpr bool isCompileTime = TestCompileTime<EnumA, ctkA::mapNameToEnum("E_0")>::value;
  EXPECT_TRUE(isCompileTime);
  EXPECT_EQ(ctkA::mapNameToEnum("E_0"), EnumA::E_0);
  EXPECT_EQ(ctkA::mapNameToEnum("E_1"), EnumA::E_1);
  EXPECT_EQ(ctkA::mapNameToEnum("E_2"), EnumA::E_2);
  EXPECT_EQ(ctkA::mapNameToEnum("E_3"), EnumA::E_3);
  EXPECT_THROW(ctkA::mapNameToEnum("SIZE"), std::out_of_range);
}

TEST(CompileTimeKeys, mapNameToId) {
  constexpr bool isCompileTime = TestCompileTime<int, ctkA::mapNameToId("E_0")>::value;
  EXPECT_TRUE(isCompileTime);
  EXPECT_EQ(ctkA::mapNameToId("E_0"), 5);
  EXPECT_EQ(ctkA::mapNameToId("E_1"), 1);
  EXPECT_EQ(ctkA::mapNameToId("E_2"), 12);
  EXPECT_EQ(ctkA::mapNameToId("E_3"), 3);
  EXPECT_THROW(ctkA::mapNameToId("SIZE"), std::out_of_range);
}

TEST(CompileTimeKeys, containsEnum) {
  constexpr bool isCompileTime = TestCompileTime<bool, ctkA::containsEnum(EnumA::E_0)>::value;
  EXPECT_TRUE(isCompileTime);
  EXPECT_TRUE(ctkA::containsEnum(EnumA::E_0));
  EXPECT_TRUE(ctkA::containsEnum(EnumA::E_1));
  EXPECT_TRUE(ctkA::containsEnum(EnumA::E_2));
  EXPECT_TRUE(ctkA::containsEnum(EnumA::E_3));
  EXPECT_FALSE(ctkA::containsEnum(EnumA::SIZE));
}

TEST(CompileTimeKeys, containsName) {
  constexpr bool isCompileTime = TestCompileTime<bool, ctkA::containsName("E_0")>::value;
  EXPECT_TRUE(isCompileTime);
  EXPECT_TRUE(ctkA::containsName("E_0"));
  EXPECT_TRUE(ctkA::containsName("E_1"));
  EXPECT_TRUE(ctkA::containsName("E_2"));
  EXPECT_TRUE(ctkA::containsName("E_3"));
  EXPECT_FALSE(ctkA::containsName("E_4"));
  EXPECT_FALSE(ctkA::containsName("BLA"));
}

TEST(CompileTimeKeys, containsId) {
  constexpr bool isCompileTime = TestCompileTime<bool, ctkA::containsId(1)>::value;
  EXPECT_TRUE(isCompileTime);
  EXPECT_TRUE(ctkA::containsId(1));
  EXPECT_TRUE(ctkA::containsId(3));
  EXPECT_TRUE(ctkA::containsId(5));
  EXPECT_TRUE(ctkA::containsId(12));
  EXPECT_FALSE(ctkA::containsId(0));
  EXPECT_FALSE(ctkA::containsId(2));
}

TEST(CompileTimeKeys, size) {
  constexpr bool isCompileTime = TestCompileTime<std::size_t, ctkA::size()>::value;
  EXPECT_TRUE(isCompileTime);
  EXPECT_EQ(1, ctkAFront::size());
  EXPECT_EQ(3, ctkABack::size());
  EXPECT_EQ(4, ctkA::size());
}

TEST(CompileTimeKeys, keysArray) {
  auto keyContainer = ctkA::getKeysArray();
  EXPECT_EQ(strcmp("E_0", keyContainer.at(EnumA::E_0).getName()), 0);
  EXPECT_EQ(5, keyContainer.atId(5).getId());
  EXPECT_EQ(strcmp("E_0", keyContainer.atId(5).getName()), 0);
  EXPECT_EQ(EnumA::E_0, keyContainer.atId(5).getEnum());
  EXPECT_EQ(1, keyContainer.at(EnumA::E_1).getId());
  EXPECT_EQ(strcmp("E_1", keyContainer.at(EnumA::E_1).getName()), 0);
  EXPECT_EQ(12, keyContainer.get<EnumA::E_2>().getId());
  EXPECT_EQ(EnumA::E_2, keyContainer.atId(12).getEnum());
  EXPECT_EQ(5, keyContainer.atName("E_0").getId());
  EXPECT_EQ(4, keyContainer.size());
}

TEST(CompileTimeKeys, concatenate) {
  // Concatenate in right order -> types should match
  using ctkAConcatenated = std_utils::ctk_concatenate_t<EnumA, ctkAFront, ctkABack>;
  constexpr bool ctkAConcatenatedSuccessful = std::is_same<ctkAConcatenated, ctkA>::value;
  EXPECT_TRUE(ctkAConcatenatedSuccessful);

  // Concatenate in wrong order -> types shouldn't match
  using ctkAConcatenatedWrongOrder = std_utils::ctk_concatenate_t<EnumA, ctkABack, ctkAFront>;
  constexpr bool ctkAConcatenatedWrongOrderSuccessful = std::is_same<ctkAConcatenatedWrongOrder, ctkA>::value;
  EXPECT_FALSE(ctkAConcatenatedWrongOrderSuccessful);

  // Contents should match anyway
  for (int i = 0; i < static_cast<int>(EnumA::SIZE); ++i) {
    const auto e = static_cast<EnumA>(i);
    EXPECT_EQ(ctkAConcatenated::mapEnumToId(e), ctkA::mapEnumToId(e));
    EXPECT_EQ(ctkAConcatenatedWrongOrder::mapEnumToId(e), ctkA::mapEnumToId(e));
    EXPECT_EQ(strcmp(ctkAConcatenated::mapEnumToName(e), ctkA::mapEnumToName(e)), 0);
    EXPECT_EQ(strcmp(ctkAConcatenatedWrongOrder::mapEnumToName(e), ctkA::mapEnumToName(e)), 0);
  }
}

TEST(CompileTimeKeys, insert_back) {
  using ctkA_ib =
      std_utils::ctk_insert_back_t<EnumA, ctkAFront, std_utils::CompileTimeKey<EnumA, EnumA::E_1, ct_string("E_1")>>;
  using ctkA_ib2 =
      std_utils::ctk_insert_back_t<EnumA, ctkA_ib, std_utils::CompileTimeKey<EnumA, EnumA::E_2, ct_string("E_2"), 12>>;
  using ctkA_ib3 =
      std_utils::ctk_insert_back_t<EnumA, ctkA_ib2, std_utils::CompileTimeKey<EnumA, EnumA::E_3, ct_string("E_3")>>;
  constexpr bool ctkABackInsertionSuccessful = std::is_same<ctkA_ib3, ctkA>::value;
  EXPECT_TRUE(ctkABackInsertionSuccessful);
}

TEST(CompileTimeKeys, insert_front) {
  using ctkA_if =
      std_utils::ctk_insert_front_t<EnumA, ctkABack, std_utils::CompileTimeKey<EnumA, EnumA::E_0, ct_string("E_0"), 5>>;
  constexpr bool ctkAFrontInsertionSuccessful = std::is_same<ctkA_if, ctkA>::value;
  EXPECT_TRUE(ctkAFrontInsertionSuccessful);
}

TEST(CompileTimeKeys, erase) {
  using ctkA_er = std_utils::ctk_erase_t<EnumA, ctkA, EnumA::E_0>;
  constexpr bool ctkAEraseSuccessful = std::is_same<ctkA_er, ctkABack>::value;
  EXPECT_TRUE(ctkAEraseSuccessful);
}

TEST(CompileTimeKeys, pop_front) {
  using ctkA_pf = std_utils::ctk_pop_front_t<EnumA, ctkA>;
  constexpr bool ctkAPopFrontSuccessful = std::is_same<ctkA_pf, ctkABack>::value;
  EXPECT_TRUE(ctkAPopFrontSuccessful);
}

TEST(CompileTimeKeys, increment_ids) {
  using ctkA_ids = std_utils::ctk_increment_ids_t<EnumA, ctkA, 3>;
  EXPECT_EQ(ctkA_ids::mapEnumToId(EnumA::E_0), 8);
  EXPECT_EQ(ctkA_ids::mapEnumToId(EnumA::E_1), 4);
  EXPECT_EQ(ctkA_ids::mapEnumToId(EnumA::E_2), 15);
  EXPECT_EQ(ctkA_ids::mapEnumToId(EnumA::E_3), 6);
  EXPECT_THROW(ctkA_ids::mapEnumToId(EnumA::SIZE), std::out_of_range);
}

TEST(CompileTimeKeys, pop_front_and_decrement) {
  using ctkA_pfd = std_utils::ctk_pop_front_and_decrement_t<EnumA, ctkA>;
  EXPECT_EQ(ctkA_pfd::mapEnumToId(EnumA::E_1), 0);
  EXPECT_EQ(ctkA_pfd::mapEnumToId(EnumA::E_2), 11);
  EXPECT_EQ(ctkA_pfd::mapEnumToId(EnumA::E_3), 2);
  EXPECT_THROW(ctkA_pfd::mapEnumToId(EnumA::E_0), std::out_of_range);
  EXPECT_THROW(ctkA_pfd::mapEnumToId(EnumA::SIZE), std::out_of_range);
}

TEST(CompileTimeKeys, transform) {
  using ctkA2_transformed = std_utils::ctk_transform_t<EnumA, ctkA2, mapEnumA2ToEnumA, 1>;
  constexpr bool ctkA2TransformSuccessful = std::is_same<ctkA2_transformed, ctkABack>::value;
  EXPECT_TRUE(ctkA2TransformSuccessful);
}

TEST(CompileTimeKeys, combine) {
  using ctkA_combined = std_utils::ctk_combine_t<EnumA, ctkA1, ctkA2, mapEnumA1ToEnumA, mapEnumA2ToEnumA>;
  constexpr bool ctkACombindedSuccessful = std::is_same<ctkA_combined, ctkA>::value;
  EXPECT_TRUE(ctkACombindedSuccessful);
}