// gtest
#include <gtest/gtest.h>

// yaml tools
#include "yaml_tools/yaml_tools.hpp"

class TestYamlNodeScalarStringEmpty : public ::testing::Test {
 protected:
  void SetUp() override { yamlNode_ = value_; }

 protected:
  using Type = std::string;
  const Type value_ = "";
  const std::string valueAsString_ = "\"\"";
  yaml_tools::YamlNode yamlNode_;
};

TEST_F(TestYamlNodeScalarStringEmpty, throwsExceptionOnBadCast) {  // NOLINT
  // string can only be casted to string.
  EXPECT_THROW(yamlNode_.as<bool>(), yaml_tools::Exception);    // NOLINT
  EXPECT_THROW(yamlNode_.as<int>(), yaml_tools::Exception);     // NOLINT
  EXPECT_THROW(yamlNode_.as<double>(), yaml_tools::Exception);  // NOLINT
  EXPECT_NO_THROW(yamlNode_.as<std::string>());                 // NOLINT
}

TEST_F(TestYamlNodeScalarStringEmpty, accessValue) {  // NOLINT
  EXPECT_EQ(value_, yamlNode_.as<std::string>());
}

TEST_F(TestYamlNodeScalarStringEmpty, writeToString) {  // NOLINT
  EXPECT_EQ(valueAsString_, yamlNode_.toString());
}

TEST_F(TestYamlNodeScalarStringEmpty, readFromString) {  // NOLINT
  EXPECT_EQ(value_, yaml_tools::YamlNode::fromString(valueAsString_).as<Type>());
}

TEST_F(TestYamlNodeScalarStringEmpty, readFromAndWriteToString) {  // NOLINT
  EXPECT_EQ(valueAsString_, yaml_tools::YamlNode::fromString(valueAsString_).toString());
}
