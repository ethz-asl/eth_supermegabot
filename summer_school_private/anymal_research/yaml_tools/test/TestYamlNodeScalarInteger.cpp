// gtest
#include <gtest/gtest.h>

// yaml tools
#include "yaml_tools/yaml_tools.hpp"

class TestYamlNodeScalarInteger : public ::testing::Test {
 protected:
  void SetUp() override { yamlNode_ = value_; }

 protected:
  using Type = int;
  const Type value_ = 1;
  const std::string valueAsString_ = "1";
  yaml_tools::YamlNode yamlNode_;
};

TEST_F(TestYamlNodeScalarInteger, throwsExceptionOnBadCast) {  // NOLINT
  // int can be casted to int, double, and string.
  EXPECT_THROW(yamlNode_.as<bool>(), yaml_tools::Exception);  // NOLINT
  EXPECT_NO_THROW(yamlNode_.as<int>());                       // NOLINT
  EXPECT_NO_THROW(yamlNode_.as<double>());                    // NOLINT
  EXPECT_NO_THROW(yamlNode_.as<std::string>());               // NOLINT
}

TEST_F(TestYamlNodeScalarInteger, accessValue) {  // NOLINT
  EXPECT_EQ(value_, yamlNode_.as<Type>());
}

TEST_F(TestYamlNodeScalarInteger, writeToString) {  // NOLINT
  EXPECT_EQ(valueAsString_, yamlNode_.toString());
}

TEST_F(TestYamlNodeScalarInteger, readFromString) {  // NOLINT
  EXPECT_EQ(value_, yaml_tools::YamlNode::fromString(valueAsString_).as<Type>());
}

TEST_F(TestYamlNodeScalarInteger, readFromAndWriteToString) {  // NOLINT
  EXPECT_EQ(valueAsString_, yaml_tools::YamlNode::fromString(valueAsString_).toString());
}
