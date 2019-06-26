// gtest
#include <gtest/gtest.h>

// yaml tools
#include "yaml_tools/yaml_tools.hpp"

class TestYamlNodeScalarDoubleNaN : public ::testing::Test {
 protected:
  void SetUp() override { yamlNode_ = value_; }

 protected:
  using Type = double;
  const Type value_ = std::numeric_limits<double>::quiet_NaN();
  const std::string valueAsString_ = ".nan";
  yaml_tools::YamlNode yamlNode_;
};

TEST_F(TestYamlNodeScalarDoubleNaN, throwsExceptionOnBadCast) {  // NOLINT
  // double can be casted to double and string.
  EXPECT_THROW(yamlNode_.as<bool>(), yaml_tools::Exception);  // NOLINT
  EXPECT_THROW(yamlNode_.as<int>(), yaml_tools::Exception);   // NOLINT
  EXPECT_NO_THROW(yamlNode_.as<double>());                    // NOLINT
  EXPECT_NO_THROW(yamlNode_.as<std::string>());               // NOLINT
}

TEST_F(TestYamlNodeScalarDoubleNaN, accessValue) {  // NOLINT
  // Note: EXPECT_EQ cannot be used, as NaN == Nan returns false.
  EXPECT_TRUE(std::isnan(value_));
  EXPECT_TRUE(std::isnan(yamlNode_.as<Type>()));
}

TEST_F(TestYamlNodeScalarDoubleNaN, writeToString) {  // NOLINT
  EXPECT_EQ(valueAsString_, yamlNode_.toString());
}

TEST_F(TestYamlNodeScalarDoubleNaN, readFromString) {  // NOLINT
  // Note: EXPECT_EQ cannot be used, as NaN == Nan returns false.
  EXPECT_TRUE(std::isnan(value_));
  EXPECT_TRUE(std::isnan(yaml_tools::YamlNode::fromString(valueAsString_).as<Type>()));
}

TEST_F(TestYamlNodeScalarDoubleNaN, readFromAndWriteToString) {  // NOLINT
  EXPECT_EQ(valueAsString_, yaml_tools::YamlNode::fromString(valueAsString_).toString());
}
