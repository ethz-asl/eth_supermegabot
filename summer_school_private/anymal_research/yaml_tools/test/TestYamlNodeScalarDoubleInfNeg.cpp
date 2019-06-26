// gtest
#include <gtest/gtest.h>

// yaml tools
#include "yaml_tools/yaml_tools.hpp"

class TestYamlNodeScalarDoubleInfNeg : public ::testing::Test {
 protected:
  void SetUp() override { yamlNode_ = value_; }

 protected:
  using Type = double;
  const Type value_ = -std::numeric_limits<double>::infinity();
  const std::string valueAsString_ = "-.inf";
  yaml_tools::YamlNode yamlNode_;
};

TEST_F(TestYamlNodeScalarDoubleInfNeg, throwsExceptionOnBadCast) {  // NOLINT
  // double can be casted to double and string.
  EXPECT_THROW(yamlNode_.as<bool>(), yaml_tools::Exception);  // NOLINT
  EXPECT_THROW(yamlNode_.as<int>(), yaml_tools::Exception);   // NOLINT
  EXPECT_NO_THROW(yamlNode_.as<double>());                    // NOLINT
  EXPECT_NO_THROW(yamlNode_.as<std::string>());               // NOLINT
}

TEST_F(TestYamlNodeScalarDoubleInfNeg, accessValue) {  // NOLINT
  EXPECT_EQ(value_, yamlNode_.as<Type>());
}

TEST_F(TestYamlNodeScalarDoubleInfNeg, writeToString) {  // NOLINT
  EXPECT_EQ(valueAsString_, yamlNode_.toString());
}

TEST_F(TestYamlNodeScalarDoubleInfNeg, readFromString) {  // NOLINT
  EXPECT_EQ(value_, yaml_tools::YamlNode::fromString(valueAsString_).as<Type>());
}

TEST_F(TestYamlNodeScalarDoubleInfNeg, readFromAndWriteToString) {  // NOLINT
  EXPECT_EQ(valueAsString_, yaml_tools::YamlNode::fromString(valueAsString_).toString());
}
