// gtest
#include <gtest/gtest.h>

// yaml tools
#include "yaml_tools/yaml_tools.hpp"

class TestYamlNodeScalarDoubleInfPos : public ::testing::Test {
 protected:
  void SetUp() override { yamlNode_ = value_; }

 protected:
  using Type = double;
  const Type value_ = std::numeric_limits<double>::infinity();
  const std::string valueAsString_ = ".inf";
  yaml_tools::YamlNode yamlNode_;
};

TEST_F(TestYamlNodeScalarDoubleInfPos, throwsExceptionOnBadCast) {  // NOLINT
  // double can be casted to double and string.
  EXPECT_THROW(yamlNode_.as<bool>(), yaml_tools::Exception);  // NOLINT
  EXPECT_THROW(yamlNode_.as<int>(), yaml_tools::Exception);   // NOLINT
  EXPECT_NO_THROW(yamlNode_.as<double>());                    // NOLINT
  EXPECT_NO_THROW(yamlNode_.as<std::string>());               // NOLINT
}

TEST_F(TestYamlNodeScalarDoubleInfPos, accessValue) {  // NOLINT
  EXPECT_EQ(value_, yamlNode_.as<Type>());
}

TEST_F(TestYamlNodeScalarDoubleInfPos, writeToString) {  // NOLINT
  EXPECT_EQ(valueAsString_, yamlNode_.toString());
}

TEST_F(TestYamlNodeScalarDoubleInfPos, readFromString) {  // NOLINT
  EXPECT_EQ(value_, yaml_tools::YamlNode::fromString(valueAsString_).as<Type>());
}

TEST_F(TestYamlNodeScalarDoubleInfPos, readFromAndWriteToString) {  // NOLINT
  EXPECT_EQ(valueAsString_, yaml_tools::YamlNode::fromString(valueAsString_).toString());
}
