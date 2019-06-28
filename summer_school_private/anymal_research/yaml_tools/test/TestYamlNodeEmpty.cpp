// gtest
#include <gtest/gtest.h>

// yaml tools
#include "yaml_tools/yaml_tools.hpp"

class TestYamlNodeEmpty : public ::testing::Test {
 protected:
  const std::string nonExistingKey_ = "non_existing_key";
  const size_t nonExistingId_ = 1000;

  yaml_tools::YamlNode yamlNode_;
};

TEST_F(TestYamlNodeEmpty, keyIsEmptyOnConstruction) {  // NOLINT
  ASSERT_TRUE(yamlNode_.getNestedKey().empty());
}

TEST_F(TestYamlNodeEmpty, typeIsSetOnConstruction) {  // NOLINT
  EXPECT_TRUE(yamlNode_.isDefined());
  EXPECT_TRUE(yamlNode_.isNull());
  EXPECT_FALSE(yamlNode_.isScalar());
  EXPECT_FALSE(yamlNode_.isSequence());
  EXPECT_FALSE(yamlNode_.isMap());
}

TEST_F(TestYamlNodeEmpty, nonConstDoesNotThrowExceptionOnBadKey) {  // NOLINT
  yaml_tools::YamlNode yamlNode;
  EXPECT_NO_THROW(yamlNode[nonExistingKey_]);  // NOLINT
}

TEST_F(TestYamlNodeEmpty, constThrowsExceptionOnBadKey) {  // NOLINT
  const yaml_tools::YamlNode yamlNodeConst;
  EXPECT_THROW(yamlNodeConst[nonExistingKey_], yaml_tools::Exception);  // NOLINT
}
