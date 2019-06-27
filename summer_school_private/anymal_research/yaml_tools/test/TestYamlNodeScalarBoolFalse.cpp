// gtest
#include <gtest/gtest.h>

// yaml tools
#include "yaml_tools/yaml_tools.hpp"

class TestYamlNodeScalarBoolFalse : public ::testing::Test {
 protected:
  void SetUp() override { yamlNode_ = value_; }

 protected:
  using Type = bool;
  const Type value_ = false;
  const std::string valueAsString_ = "false";
  yaml_tools::YamlNode yamlNode_;
};

TEST_F(TestYamlNodeScalarBoolFalse, throwsExceptionOnBadCast) {  // NOLINT
  // bool can be casted to bool and string.
  EXPECT_NO_THROW(yamlNode_.as<bool>());                        // NOLINT
  EXPECT_THROW(yamlNode_.as<int>(), yaml_tools::Exception);     // NOLINT
  EXPECT_THROW(yamlNode_.as<double>(), yaml_tools::Exception);  // NOLINT
  EXPECT_NO_THROW(yamlNode_.as<std::string>());                 // NOLINT
}

TEST_F(TestYamlNodeScalarBoolFalse, accessValue) {  // NOLINT
  EXPECT_EQ(value_, yamlNode_.as<Type>());
}

TEST_F(TestYamlNodeScalarBoolFalse, writeToString) {  // NOLINT
  EXPECT_EQ(valueAsString_, yamlNode_.toString());
}

TEST_F(TestYamlNodeScalarBoolFalse, readFromString) {  // NOLINT
  EXPECT_EQ(value_, yaml_tools::YamlNode::fromString(valueAsString_).as<Type>());
}

TEST_F(TestYamlNodeScalarBoolFalse, readFromAndWriteToString) {  // NOLINT
  EXPECT_EQ(valueAsString_, yaml_tools::YamlNode::fromString(valueAsString_).toString());
}
