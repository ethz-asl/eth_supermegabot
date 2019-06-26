// gtest
#include <gtest/gtest.h>

// yaml tools
#include "yaml_tools/yaml_tools.hpp"

class TestYamlNodeScalar : public ::testing::Test {
 protected:
  void SetUp() override { yamlNode_ = double_; }

 protected:
  yaml_tools::YamlNode yamlNode_;
  const double double_ = 1.0;
  const double otherDouble_ = 2.3;
  const std::string string_ = "hello";
};

TEST_F(TestYamlNodeScalar, typeIsSetOnConstruction) {  // NOLINT
  EXPECT_TRUE(yamlNode_.isDefined());
  EXPECT_FALSE(yamlNode_.isNull());
  EXPECT_TRUE(yamlNode_.isScalar());
  EXPECT_FALSE(yamlNode_.isSequence());
  EXPECT_FALSE(yamlNode_.isMap());
}

TEST_F(TestYamlNodeScalar, getSize) {  // NOLINT
  EXPECT_EQ(0u, yamlNode_.size());
}

TEST_F(TestYamlNodeScalar, changeType) {  // NOLINT
  EXPECT_EQ(double_, yamlNode_.as<double>());
  yamlNode_ = string_;
  EXPECT_EQ(string_, yamlNode_.as<std::string>());
}

TEST_F(TestYamlNodeScalar, copyConstructor) {  // NOLINT
  yaml_tools::YamlNode yamlNodeCopy(yamlNode_);
  EXPECT_EQ(double_, yamlNodeCopy.as<double>());
}

TEST_F(TestYamlNodeScalar, copyConstructorIsShallow) {  // NOLINT
  yaml_tools::YamlNode yamlNodeCopy(yamlNode_);
  yamlNodeCopy = otherDouble_;
  EXPECT_EQ(otherDouble_, yamlNode_.as<double>());
}

TEST_F(TestYamlNodeScalar, assignment) {  // NOLINT
  yaml_tools::YamlNode yamlNodeCopy;
  yamlNodeCopy = yamlNode_;
  EXPECT_EQ(double_, yamlNodeCopy.as<double>());
}

TEST_F(TestYamlNodeScalar, assignmentIsShallow) {  // NOLINT
  yaml_tools::YamlNode yamlNodeCopy;
  yamlNodeCopy = yamlNode_;
  yamlNodeCopy = otherDouble_;
  EXPECT_EQ(otherDouble_, yamlNode_.as<double>());
}
