// gtest
#include <gtest/gtest.h>

// yaml tools
#include "yaml_tools/YamlNode.hpp"

class TestYamlNodeScalarStringSpecialCharacters : public ::testing::Test {
 protected:
  using Type = std::string;
  struct Character {
    Type value_;
    std::string valueAsString_;
    yaml_tools::YamlNode yamlNode_;
    Character(Type value, std::string valueAsString) : value_(std::move(value)), valueAsString_(std::move(valueAsString)) {
      yamlNode_ = value_;
    }
  };
  // Note: A single " does not work.
  const std::vector<Character> characters_ = {
      Character("+", "+"),     Character("-", "\"-\""), Character("=", "="),     Character("[", "\"[\""), Character("]", "\"]\""),
      Character("{", "\"{\""), Character("}", "\"}\""), Character("<", "<"),     Character(">", "\">\""), Character("'", "\"'\""),
      Character("`", "\"`\""), Character("\\", "\\"),   Character("|", "\"|\""), Character("/", "/"),     Character(",", "\",\""),
      Character(".", "."),     Character(";", ";"),     Character(":", "\":\""), Character("!", "\"!\""), Character("?", "\"?\""),
      Character("_", "_"),     Character("~", "~"),     Character("@", "\"@\""), Character("#", "\"#\""), Character("$", "$"),
      Character("%", "\"%\""), Character("^", "^"),     Character("&", "\"&\""), Character("*", "\"*\""), Character("°", "°"),
      Character("^", "^"),     Character(" ", "\" \"")};
};

TEST_F(TestYamlNodeScalarStringSpecialCharacters, throwsExceptionOnBadCast) {  // NOLINT
  // string can only be casted to string.
  for (const auto& character : characters_) {
    EXPECT_THROW(character.yamlNode_.as<bool>(), yaml_tools::Exception);    // NOLINT
    EXPECT_THROW(character.yamlNode_.as<int>(), yaml_tools::Exception);     // NOLINT
    EXPECT_THROW(character.yamlNode_.as<double>(), yaml_tools::Exception);  // NOLINT
    EXPECT_NO_THROW(character.yamlNode_.as<std::string>());                 // NOLINT
  }
}

TEST_F(TestYamlNodeScalarStringSpecialCharacters, accessValue) {  // NOLINT
  for (const auto& character : characters_) {
    EXPECT_EQ(character.value_, character.yamlNode_.as<std::string>());
  }
}

TEST_F(TestYamlNodeScalarStringSpecialCharacters, writeToString) {  // NOLINT
  for (const auto& character : characters_) {
    EXPECT_EQ(character.valueAsString_, character.yamlNode_.toString());
  }
}

TEST_F(TestYamlNodeScalarStringSpecialCharacters, readFromString) {  // NOLINT
  for (const auto& character : characters_) {
    EXPECT_EQ(character.value_, yaml_tools::YamlNode::fromString(character.valueAsString_).as<Type>());
  }
}

TEST_F(TestYamlNodeScalarStringSpecialCharacters, readFromAndWriteToString) {  // NOLINT
  for (const auto& character : characters_) {
    EXPECT_EQ(character.valueAsString_, yaml_tools::YamlNode::fromString(character.valueAsString_).toString());
  }
}
