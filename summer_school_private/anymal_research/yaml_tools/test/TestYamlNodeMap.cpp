// gtest
#include <gtest/gtest.h>

// yaml tools
#include "yaml_tools/yaml_tools.hpp"

class TestYamlNodeMap : public ::testing::Test {
 protected:
  void SetUp() override {
    yamlNode_[bool_.first] = bool_.second;
    yamlNode_[integer_.first] = integer_.second;
    yamlNode_[double_.first] = double_.second;
    yamlNode_[string_.first] = string_.second;
  }

 protected:
  const std::pair<std::string, bool> bool_ = {"bool", true};
  const std::pair<std::string, int> integer_ = {"integer", 1};
  const std::pair<std::string, double> double_ = {"double", 2.3};
  const std::pair<std::string, std::string> string_ = {"string", "hello"};
  const std::string goodKey_ = "double";
  const std::string badKey_ = "non_existing_key";
  const size_t anyId_ = 1000;

  const std::string yamlString_ =  // clang-format off
      "bool: true\n"
      "integer: 1\n"
      "double: 2.3\n"
      "string: hello";  // clang-format on
  const std::string yamlStringAlphabetical_ =  // clang-format off
      "bool: true\n"
      "double: 2.3\n"
      "integer: 1\n"
      "string: hello";  // clang-format on

  yaml_tools::YamlNode yamlNode_;
};

TEST_F(TestYamlNodeMap, readFromString) {  // NOLINT
  EXPECT_EQ(yamlNode_, yaml_tools::YamlNode::fromString(yamlString_));
}

TEST_F(TestYamlNodeMap, writeToString) {  // NOLINT
  EXPECT_EQ(yamlStringAlphabetical_, yamlNode_.toString());
}

TEST_F(TestYamlNodeMap, typeIsSetOnConstruction) {  // NOLINT
  EXPECT_TRUE(yamlNode_.isDefined());
  EXPECT_FALSE(yamlNode_.isNull());
  EXPECT_FALSE(yamlNode_.isScalar());
  EXPECT_FALSE(yamlNode_.isSequence());
  EXPECT_TRUE(yamlNode_.isMap());
}

TEST_F(TestYamlNodeMap, getSize) {  // NOLINT
  EXPECT_EQ(4u, yamlNode_.size());
}

TEST_F(TestYamlNodeMap, nonConstDoesNotThrowExceptionOnGoodKey) {  // NOLINT
  EXPECT_NO_THROW(yamlNode_[goodKey_]);                            // NOLINT
}

TEST_F(TestYamlNodeMap, constDoesNotThrowExceptionOnGoodKey) {  // NOLINT
  const yaml_tools::YamlNode constYamlNode = yamlNode_;
  EXPECT_NO_THROW(constYamlNode[goodKey_]);  // NOLINT
}

TEST_F(TestYamlNodeMap, nonConstDoesNotThrowExceptionOnBadKey) {  // NOLINT
  EXPECT_NO_THROW(yamlNode_[badKey_]);                            // NOLINT
}

TEST_F(TestYamlNodeMap, constThrowsExceptionOnBadKey) {  // NOLINT
  const yaml_tools::YamlNode constYamlNode = yamlNode_;
  EXPECT_THROW(constYamlNode[badKey_], yaml_tools::Exception);  // NOLINT
}

TEST_F(TestYamlNodeMap, nonConstThrowsExceptionOnId) {     // NOLINT
  EXPECT_THROW(yamlNode_[anyId_], yaml_tools::Exception);  // NOLINT
}

TEST_F(TestYamlNodeMap, constThrowsExceptionOnId) {  // NOLINT
  const yaml_tools::YamlNode constYamlNode = yamlNode_;
  EXPECT_THROW(constYamlNode[anyId_], yaml_tools::Exception);  // NOLINT
}

TEST_F(TestYamlNodeMap, memberAccessAndCasting) {  // NOLINT
  EXPECT_EQ(bool_.second, yamlNode_[bool_.first].as<bool>());
  EXPECT_EQ(integer_.second, yamlNode_[integer_.first].as<int>());
  EXPECT_EQ(double_.second, yamlNode_[double_.first].as<double>());
  EXPECT_EQ(string_.second, yamlNode_[string_.first].as<std::string>());
}

TEST_F(TestYamlNodeMap, iteratorBasedForLoop) {  // NOLINT
  unsigned int i = 0;
  for (auto element = yamlNode_.begin(); element != yamlNode_.end(); element++) {
    const std::string key = element->first;
    const yaml_tools::YamlNode& value = element->second;
    // Elements must be alphabetic.
    switch (i) {
      case 0:
        EXPECT_EQ(bool_.first, key);
        EXPECT_EQ(bool_.second, value.as<bool>());
        EXPECT_EQ("/bool", value.getNestedKey());
        break;
      case 1:
        EXPECT_EQ(double_.first, key);
        EXPECT_EQ(double_.second, value.as<double>());
        EXPECT_EQ("/double", value.getNestedKey());
        break;
      case 2:
        EXPECT_EQ(integer_.first, key);
        EXPECT_EQ(integer_.second, value.as<int>());
        EXPECT_EQ("/integer", value.getNestedKey());
        break;
      case 3:
        EXPECT_EQ(string_.first, key);
        EXPECT_EQ(string_.second, value.as<std::string>());
        EXPECT_EQ("/string", value.getNestedKey());
        break;
      default:
        // We should never arrive here.
        ASSERT_TRUE(false);
    }
    i++;
  }
}

TEST_F(TestYamlNodeMap, constIteratorBasedForLoop) {  // NOLINT
  const yaml_tools::YamlNode yamlNode = yamlNode_;
  unsigned int i = 0;
  for (auto element = yamlNode.begin(); element != yamlNode.end(); element++) {
    const std::string key = element->first;
    const yaml_tools::YamlNode& value = element->second;
    // Elements must be alphabetic.
    switch (i) {
      case 0:
        EXPECT_EQ(bool_.first, key);
        EXPECT_EQ(bool_.second, value.as<bool>());
        EXPECT_EQ("/bool", value.getNestedKey());
        break;
      case 1:
        EXPECT_EQ(double_.first, key);
        EXPECT_EQ(double_.second, value.as<double>());
        EXPECT_EQ("/double", value.getNestedKey());
        break;
      case 2:
        EXPECT_EQ(integer_.first, key);
        EXPECT_EQ(integer_.second, value.as<int>());
        EXPECT_EQ("/integer", value.getNestedKey());
        break;
      case 3:
        EXPECT_EQ(string_.first, key);
        EXPECT_EQ(string_.second, value.as<std::string>());
        EXPECT_EQ("/string", value.getNestedKey());
        break;
      default:
        // We should never arrive here.
        ASSERT_TRUE(false);
    }
    i++;
  }
}

TEST_F(TestYamlNodeMap, rangeBasedForLoop) {  // NOLINT
  unsigned int i = 0;
  for (auto element : yamlNode_) {
    const std::string key = element.first;
    const yaml_tools::YamlNode& value = element.second;
    // Elements must be alphabetic.
    switch (i) {
      case 0:
        EXPECT_EQ(bool_.first, key);
        EXPECT_EQ(bool_.second, value.as<bool>());
        EXPECT_EQ("/bool", value.getNestedKey());
        break;
      case 1:
        EXPECT_EQ(double_.first, key);
        EXPECT_EQ(double_.second, value.as<double>());
        EXPECT_EQ("/double", value.getNestedKey());
        break;
      case 2:
        EXPECT_EQ(integer_.first, key);
        EXPECT_EQ(integer_.second, value.as<int>());
        EXPECT_EQ("/integer", value.getNestedKey());
        break;
      case 3:
        EXPECT_EQ(string_.first, key);
        EXPECT_EQ(string_.second, value.as<std::string>());
        EXPECT_EQ("/string", value.getNestedKey());
        break;
      default:
        // We should never arrive here.
        ASSERT_TRUE(false);
    }
    i++;
  }
}

TEST_F(TestYamlNodeMap, constRangeBasedForLoop) {  // NOLINT
  const yaml_tools::YamlNode yamlNode = yamlNode_;
  unsigned int i = 0;
  for (auto element : yamlNode) {
    const std::string key = element.first;
    const yaml_tools::YamlNode& value = element.second;
    // Elements must be alphabetic.
    switch (i) {
      case 0:
        EXPECT_EQ(bool_.first, key);
        EXPECT_EQ(bool_.second, value.as<bool>());
        EXPECT_EQ("/bool", value.getNestedKey());
        break;
      case 1:
        EXPECT_EQ(double_.first, key);
        EXPECT_EQ(double_.second, value.as<double>());
        EXPECT_EQ("/double", value.getNestedKey());
        break;
      case 2:
        EXPECT_EQ(integer_.first, key);
        EXPECT_EQ(integer_.second, value.as<int>());
        EXPECT_EQ("/integer", value.getNestedKey());
        break;
      case 3:
        EXPECT_EQ(string_.first, key);
        EXPECT_EQ(string_.second, value.as<std::string>());
        EXPECT_EQ("/string", value.getNestedKey());
        break;
      default:
        // We should never arrive here.
        ASSERT_TRUE(false);
    }
    i++;
  }
}
