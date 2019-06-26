// gtest
#include <gtest/gtest.h>

// yaml tools
#include "yaml_tools/yaml_tools.hpp"

class TestYamlNodeSequence : public ::testing::Test {
 protected:
  void SetUp() override {
    yaml_tools::YamlNode yamlNodeBool;
    yamlNodeBool["name"] = bool_.first;
    yamlNodeBool["value"] = bool_.second;
    yamlNode_.pushBack(yamlNodeBool);
    yaml_tools::YamlNode yamlNodeInteger;
    yamlNodeInteger["name"] = integer_.first;
    yamlNodeInteger["value"] = integer_.second;
    yamlNode_.pushBack(yamlNodeInteger);
    yaml_tools::YamlNode yamlNodeDouble;
    yamlNodeDouble["name"] = double_.first;
    yamlNodeDouble["value"] = double_.second;
    yamlNode_.pushBack(yamlNodeDouble);
    yaml_tools::YamlNode yamlNodeString;
    yamlNodeString["name"] = string_.first;
    yamlNodeString["value"] = string_.second;
    yamlNode_.pushBack(yamlNodeString);
  }

 protected:
  const std::pair<std::string, bool> bool_ = {"bool", true};
  const std::pair<std::string, int> integer_ = {"integer", 1};
  const std::pair<std::string, double> double_ = {"double", 4.5};
  const std::pair<std::string, std::string> string_ = {"string", "hello"};
  const size_t goodId_ = 2;
  const size_t badId_ = 1000;
  const std::string anyKey_ = "any_key";

  const std::string yamlString_ =  // clang-format off
      "- name: bool\n"
      "  value: true\n"
      "- name: integer\n"
      "  value: 1\n"
      "- name: double\n"
      "  value: 4.5\n"
      "- name: string\n"
      "  value: hello";  // clang-format on

  yaml_tools::YamlNode yamlNode_;
};

TEST_F(TestYamlNodeSequence, readFromString) {  // NOLINT
  EXPECT_EQ(yamlNode_, yaml_tools::YamlNode::fromString(yamlString_));
}

TEST_F(TestYamlNodeSequence, writeToString) {  // NOLINT
  EXPECT_EQ(yamlString_, yamlNode_.toString());
}

TEST_F(TestYamlNodeSequence, typeIsSetOnConstruction) {  // NOLINT
  EXPECT_TRUE(yamlNode_.isDefined());
  EXPECT_FALSE(yamlNode_.isNull());
  EXPECT_FALSE(yamlNode_.isScalar());
  EXPECT_TRUE(yamlNode_.isSequence());
  EXPECT_FALSE(yamlNode_.isMap());
}

TEST_F(TestYamlNodeSequence, getSize) {  // NOLINT
  EXPECT_EQ(4u, yamlNode_.size());
}

TEST_F(TestYamlNodeSequence, nonConstNodeDoesNotThrowExceptionOnGoodId) {  // NOLINT
  EXPECT_NO_THROW(yamlNode_[goodId_]);                                     // NOLINT
}

TEST_F(TestYamlNodeSequence, constNodeDoesNotThrowExceptionOnGoodId) {  // NOLINT
  const yaml_tools::YamlNode constYamlNode = yamlNode_;
  EXPECT_NO_THROW(constYamlNode[goodId_]);  // NOLINT
}

TEST_F(TestYamlNodeSequence, nonConstNodeThrowsExceptionOnBadId) {  // NOLINT
  EXPECT_THROW(yamlNode_[badId_], yaml_tools::Exception);           // NOLINT
}

TEST_F(TestYamlNodeSequence, constNodeThrowsExceptionOnBadId) {  // NOLINT
  const yaml_tools::YamlNode constYamlNode = yamlNode_;
  EXPECT_THROW(constYamlNode[badId_], yaml_tools::Exception);  // NOLINT
}

TEST_F(TestYamlNodeSequence, nonConstNodeThrowsExceptionOnKey) {  // NOLINT
  // Note: It is surprising that accessing a sequence in a way one would access a map actually turns it into a map.
  EXPECT_TRUE(yamlNode_.isSequence());
  EXPECT_FALSE(yamlNode_.isMap());
  EXPECT_NO_THROW(yamlNode_[anyKey_]);  // NOLINT
  EXPECT_FALSE(yamlNode_.isSequence());
  EXPECT_TRUE(yamlNode_.isMap());
}

TEST_F(TestYamlNodeSequence, constNodeThrowsExceptionOnKey) {  // NOLINT
  const yaml_tools::YamlNode constYamlNode = yamlNode_;
  EXPECT_THROW(constYamlNode[anyKey_], yaml_tools::Exception);  // NOLINT
}

TEST_F(TestYamlNodeSequence, indexBaseForLoop) {  // NOLINT
  for (size_t i = 0; i < yamlNode_.size(); i++) {
    const yaml_tools::YamlNode& element = yamlNode_[i];
    const std::string name = element["name"].as<std::string>();
    const yaml_tools::YamlNode& value = element["value"];
    switch (i) {
      case 0:
        EXPECT_EQ(bool_.first, name);
        EXPECT_EQ(bool_.second, value.as<bool>());
        EXPECT_EQ("/0/value", value.getNestedKey());
        break;
      case 1:
        EXPECT_EQ(integer_.first, name);
        EXPECT_EQ(integer_.second, value.as<int>());
        EXPECT_EQ("/1/value", value.getNestedKey());
        break;
      case 2:
        EXPECT_EQ(double_.first, name);
        EXPECT_EQ(double_.second, value.as<double>());
        EXPECT_EQ("/2/value", value.getNestedKey());
        break;
      case 3:
        EXPECT_EQ(string_.first, name);
        EXPECT_EQ(string_.second, value.as<std::string>());
        EXPECT_EQ("/3/value", value.getNestedKey());
        break;
      default:
        // We should never arrive here.
        ASSERT_TRUE(false);
    }
  }
}

TEST_F(TestYamlNodeSequence, iteratorBasedForLoop) {  // NOLINT
  unsigned int i = 0;
  for (auto it = yamlNode_.begin(); it != yamlNode_.end(); it++) {
    const yaml_tools::YamlNode& element = *it;
    const std::string name = element["name"].as<std::string>();
    const yaml_tools::YamlNode& value = element["value"];
    switch (i) {
      case 0:
        EXPECT_EQ(bool_.first, name);
        EXPECT_EQ(bool_.second, value.as<bool>());
        EXPECT_EQ("/0/value", value.getNestedKey());
        break;
      case 1:
        EXPECT_EQ(integer_.first, name);
        EXPECT_EQ(integer_.second, value.as<int>());
        EXPECT_EQ("/1/value", value.getNestedKey());
        break;
      case 2:
        EXPECT_EQ(double_.first, name);
        EXPECT_EQ(double_.second, value.as<double>());
        EXPECT_EQ("/2/value", value.getNestedKey());
        break;
      case 3:
        EXPECT_EQ(string_.first, name);
        EXPECT_EQ(string_.second, value.as<std::string>());
        EXPECT_EQ("/3/value", value.getNestedKey());
        break;
      default:
        // We should never arrive here.
        ASSERT_TRUE(false);
    }
    i++;
  }
}

TEST_F(TestYamlNodeSequence, constIteratorBasedForLoop) {  // NOLINT
  const auto yamlNode = yamlNode_;
  unsigned int i = 0;
  for (auto it = yamlNode.begin(); it != yamlNode.end(); it++) {
    const yaml_tools::YamlNode& element = *it;
    const std::string name = element["name"].as<std::string>();
    const yaml_tools::YamlNode& value = element["value"];
    switch (i) {
      case 0:
        EXPECT_EQ(bool_.first, name);
        EXPECT_EQ(bool_.second, value.as<bool>());
        EXPECT_EQ("/0/value", value.getNestedKey());
        break;
      case 1:
        EXPECT_EQ(integer_.first, name);
        EXPECT_EQ(integer_.second, value.as<int>());
        EXPECT_EQ("/1/value", value.getNestedKey());
        break;
      case 2:
        EXPECT_EQ(double_.first, name);
        EXPECT_EQ(double_.second, value.as<double>());
        EXPECT_EQ("/2/value", value.getNestedKey());
        break;
      case 3:
        EXPECT_EQ(string_.first, name);
        EXPECT_EQ(string_.second, value.as<std::string>());
        EXPECT_EQ("/3/value", value.getNestedKey());
        break;
      default:
        // We should never arrive here.
        ASSERT_TRUE(false);
    }
    i++;
  }
}

TEST_F(TestYamlNodeSequence, rangeBasedForLoop) {  // NOLINT
  unsigned int i = 0;
  for (auto element : yamlNode_) {
    const std::string name = element["name"].as<std::string>();
    const yaml_tools::YamlNode& value = element["value"];
    switch (i) {
      case 0:
        EXPECT_EQ(bool_.first, name);
        EXPECT_EQ(bool_.second, value.as<bool>());
        EXPECT_EQ("/0/value", value.getNestedKey());
        break;
      case 1:
        EXPECT_EQ(integer_.first, name);
        EXPECT_EQ(integer_.second, value.as<int>());
        EXPECT_EQ("/1/value", value.getNestedKey());
        break;
      case 2:
        EXPECT_EQ(double_.first, name);
        EXPECT_EQ(double_.second, value.as<double>());
        EXPECT_EQ("/2/value", value.getNestedKey());
        break;
      case 3:
        EXPECT_EQ(string_.first, name);
        EXPECT_EQ(string_.second, value.as<std::string>());
        EXPECT_EQ("/3/value", value.getNestedKey());
        break;
      default:
        // We should never arrive here.
        ASSERT_TRUE(false);
    }
    i++;
  }
}

TEST_F(TestYamlNodeSequence, constRangeBasedForLoop) {  // NOLINT
  const auto yamlNode = yamlNode_;
  unsigned int i = 0;
  for (const auto& element : yamlNode) {
    const std::string name = element["name"].as<std::string>();
    const yaml_tools::YamlNode& value = element["value"];
    switch (i) {
      case 0:
        EXPECT_EQ(bool_.first, name);
        EXPECT_EQ(bool_.second, value.as<bool>());
        EXPECT_EQ("/0/value", value.getNestedKey());
        break;
      case 1:
        EXPECT_EQ(integer_.first, name);
        EXPECT_EQ(integer_.second, value.as<int>());
        EXPECT_EQ("/1/value", value.getNestedKey());
        break;
      case 2:
        EXPECT_EQ(double_.first, name);
        EXPECT_EQ(double_.second, value.as<double>());
        EXPECT_EQ("/2/value", value.getNestedKey());
        break;
      case 3:
        EXPECT_EQ(string_.first, name);
        EXPECT_EQ(string_.second, value.as<std::string>());
        EXPECT_EQ("/3/value", value.getNestedKey());
        break;
      default:
        // We should never arrive here.
        ASSERT_TRUE(false);
    }
    i++;
  }
}
