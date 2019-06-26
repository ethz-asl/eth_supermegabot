// gtest
#include <gtest/gtest.h>

// yaml tools
#include "yaml_tools/yaml_tools.hpp"

class TestYamlNodeNested : public ::testing::Test {
 protected:
  void SetUp() override {
    // Set up a map of a sequence of maps in an uncommon way.
    yaml_tools::YamlNode yamlNodeMap;
    yamlNodeMapRoot_[mapKeyRoot_] = yamlNodeMap;

    // Add map 0 to sequence.
    yaml_tools::YamlNode yamlNodeMap0;
    yamlNodeMap.pushBack(yamlNodeMap0);
    yamlNodeMap[0][bool0_.first] = bool0_.second;
    yamlNodeMap[0][integer0_.first] = integer0_.second;
    yamlNodeMapRoot_[mapKeyRoot_][0][double0_.first] = double0_.second;
    yamlNodeMapRoot_[mapKeyRoot_][0][string0_.first] = string0_.second;

    // Add map 1 to sequence.
    yaml_tools::YamlNode yamlNodeMap1;
    yamlNodeMap1[bool1_.first] = bool1_.second;
    yamlNodeMap1[integer1_.first] = integer1_.second;
    yamlNodeMap.pushBack(yamlNodeMap1);
    yamlNodeMap1[double1_.first] = double1_.second;
    yamlNodeMap1[string1_.first] = string1_.second;
  }

 protected:
  const std::string mapKeyRoot_ = "map";

  const std::pair<std::string, bool> bool0_ = {"bool", true};
  const std::pair<std::string, int> integer0_ = {"integer", 99};
  const std::pair<std::string, double> double0_ = {"double", 2.3456789};
  const std::pair<std::string, std::string> string0_ = {"string", "hello"};

  const std::pair<std::string, bool> bool1_ = {"bool", false};
  const std::pair<std::string, int> integer1_ = {"integer", -100};
  const std::pair<std::string, double> double1_ = {"double", -7.87e-3};
  const std::pair<std::string, std::string> string1_ = {"string", "good bye"};

  yaml_tools::YamlNode yamlNodeMapRoot_;
};

TEST_F(TestYamlNodeNested, nestedMemberAccessAndCasting) {  // NOLINT
  EXPECT_EQ(bool0_.second, yamlNodeMapRoot_[mapKeyRoot_][0][bool0_.first].as<bool>());
  EXPECT_EQ(integer0_.second, yamlNodeMapRoot_[mapKeyRoot_][0][integer0_.first].as<int>());
  EXPECT_EQ(double0_.second, yamlNodeMapRoot_[mapKeyRoot_][0][double0_.first].as<double>());
  EXPECT_EQ(string0_.second, yamlNodeMapRoot_[mapKeyRoot_][0][string0_.first].as<std::string>());

  EXPECT_EQ(bool1_.second, yamlNodeMapRoot_[mapKeyRoot_][1][bool1_.first].as<bool>());
  EXPECT_EQ(integer1_.second, yamlNodeMapRoot_[mapKeyRoot_][1][integer1_.first].as<int>());
  EXPECT_EQ(double1_.second, yamlNodeMapRoot_[mapKeyRoot_][1][double1_.first].as<double>());
  EXPECT_EQ(string1_.second, yamlNodeMapRoot_[mapKeyRoot_][1][string1_.first].as<std::string>());
}

TEST_F(TestYamlNodeNested, nestedKeys) {  // NOLINT
  EXPECT_EQ("/map/0/bool", yamlNodeMapRoot_[mapKeyRoot_][0][bool0_.first].getNestedKey());
  EXPECT_EQ("/map/0/integer", yamlNodeMapRoot_[mapKeyRoot_][0][integer0_.first].getNestedKey());
  EXPECT_EQ("/map/0/double", yamlNodeMapRoot_[mapKeyRoot_][0][double0_.first].getNestedKey());
  EXPECT_EQ("/map/0/string", yamlNodeMapRoot_[mapKeyRoot_][0][string0_.first].getNestedKey());

  EXPECT_EQ("/map/1/bool", yamlNodeMapRoot_[mapKeyRoot_][1][bool1_.first].getNestedKey());
  EXPECT_EQ("/map/1/integer", yamlNodeMapRoot_[mapKeyRoot_][1][integer1_.first].getNestedKey());
  EXPECT_EQ("/map/1/double", yamlNodeMapRoot_[mapKeyRoot_][1][double1_.first].getNestedKey());
  EXPECT_EQ("/map/1/string", yamlNodeMapRoot_[mapKeyRoot_][1][string1_.first].getNestedKey());
}
