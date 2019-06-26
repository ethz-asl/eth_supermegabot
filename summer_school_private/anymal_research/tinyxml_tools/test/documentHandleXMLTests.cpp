/*!
 * @file     writeXMLTests.cpp
 * @author   Gabriel Hottiger
 * @date     Jan, 2018
 * @brief
 */

#include "tinyxml_tools/tinyxml_tools.hpp"

#include <Eigen/Core>

#include <gtest/gtest.h>

#include <array>
#include <exception>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

struct DocumentHandleXMLFixture : public ::testing::Test {
  static void SetUpTestCase() {
    ASSERT_TRUE(xmlDocumentHandleMerge1_->create("xml_files/DefaultParams.xml", tinyxml_tools::DocumentMode::APPEND));
    ASSERT_TRUE(xmlDocumentHandleMerge2_->create("xml_files/Params2.xml", tinyxml_tools::DocumentMode::APPEND));
    ASSERT_TRUE(xmlDocumentHandleMerge3_->create("xml_files/Params3.xml", tinyxml_tools::DocumentMode::APPEND));
    ASSERT_FALSE(xmlDocumentHandleMultipleRoots_->create("xml_files/MultipleRoots.xml", tinyxml_tools::DocumentMode::APPEND));
  }

  static void TearDownTestCase() {
    ASSERT_TRUE(xmlDocumentHandleMerge1_->saveAs("xml_files/MergedParams.xml", true));
    ASSERT_TRUE(xmlDocumentHandleMerge2_->saveAs("xml_files/MergedParams2.xml", true));
    ASSERT_TRUE(xmlDocumentHandleMerge3_->saveAs("xml_files/MergedParams3.xml", true));
  }

  //! XML document (see xmlDocumentHandle_)
  static std::unique_ptr<tinyxml_tools::DocumentHandleXML> xmlDocumentHandleMerge1_;
  static std::unique_ptr<tinyxml_tools::DocumentHandleXML> xmlDocumentHandleMerge2_;
  static std::unique_ptr<tinyxml_tools::DocumentHandleXML> xmlDocumentHandleMerge3_;
  static std::unique_ptr<tinyxml_tools::DocumentHandleXML> xmlDocumentHandleMultipleRoots_;

  void testMerge(std::unique_ptr<tinyxml_tools::DocumentHandleXML>& docHandle) {
    // Test seeking
    ASSERT_TRUE(docHandle->seek("Complex"));
    int val = 0;
    ASSERT_TRUE(docHandle->read("Tree", val, "tree"));
    ASSERT_EQ(val, 5);

    // Test relative seeking
    ASSERT_TRUE(docHandle->seek("../"));
    ASSERT_TRUE(docHandle->read("Values", val, "a"));
    ASSERT_EQ(val, 5);
    ASSERT_TRUE(docHandle->seek("Values"));
    ASSERT_TRUE(docHandle->readAttributes(val, "a"));
    ASSERT_EQ(val, 5);

    // Test included
    ASSERT_TRUE(docHandle->seekFromRoot("Complex/Tree"));
    std::vector<std::string> characters;
    ASSERT_TRUE(docHandle->readSiblings("Second/Apple", characters, "p"));
    ASSERT_EQ(characters[0], "b");
    ASSERT_EQ(characters[1], "c");
    ASSERT_TRUE(docHandle->seek("Second/Apple"));
    ASSERT_TRUE(docHandle->readSiblingAttributes(characters, "p"));
    ASSERT_EQ(characters[0], "b");
    ASSERT_EQ(characters[1], "c");

    // Test included
    ASSERT_TRUE(docHandle->seekFromRoot("Complex/Structure"));
    ASSERT_TRUE(docHandle->read("Tree", val, "s"));
    ASSERT_EQ(val, 7);
    ASSERT_TRUE(docHandle->seek("NewParam/File/TMP"));
    double gain;
    ASSERT_TRUE(docHandle->readAttributes(gain, "gain"));
    ASSERT_EQ(gain, 1e-3);

    // Test included
    ASSERT_TRUE(docHandle->seekFromRoot(""));
    ASSERT_TRUE(docHandle->writeSiblings("AppleCopy", characters, "c"));
  }

  void testExtension(std::unique_ptr<tinyxml_tools::DocumentHandleXML>& docHandle) {
    ASSERT_TRUE(docHandle->seekFromRoot("Complex/ExtensionValue"));
    double extensionValue;
    ASSERT_TRUE(docHandle->readAttributes(extensionValue, "x"));
    ASSERT_EQ(extensionValue, 17);
  }
};

std::unique_ptr<tinyxml_tools::DocumentHandleXML> DocumentHandleXMLFixture::xmlDocumentHandleMerge1_ =
    std::unique_ptr<tinyxml_tools::DocumentHandleXML>(new tinyxml_tools::DocumentHandleXML());
std::unique_ptr<tinyxml_tools::DocumentHandleXML> DocumentHandleXMLFixture::xmlDocumentHandleMerge2_ =
    std::unique_ptr<tinyxml_tools::DocumentHandleXML>(new tinyxml_tools::DocumentHandleXML());
std::unique_ptr<tinyxml_tools::DocumentHandleXML> DocumentHandleXMLFixture::xmlDocumentHandleMerge3_ =
    std::unique_ptr<tinyxml_tools::DocumentHandleXML>(new tinyxml_tools::DocumentHandleXML());
std::unique_ptr<tinyxml_tools::DocumentHandleXML> DocumentHandleXMLFixture::xmlDocumentHandleMultipleRoots_ =
    std::unique_ptr<tinyxml_tools::DocumentHandleXML>(new tinyxml_tools::DocumentHandleXML());

TEST_F(DocumentHandleXMLFixture, mergeParams) {  // NOLINT
  ASSERT_TRUE(xmlDocumentHandleMerge1_->merge("xml_files/Params.xml", tinyxml_tools::MergeMode::REPLACE_OVERWRITE));
}

TEST_F(DocumentHandleXMLFixture, checkMergeParams1) {  // NOLINT
  testMerge(xmlDocumentHandleMerge1_);
}

TEST_F(DocumentHandleXMLFixture, checkMergeParams2) {  // NOLINT
  testMerge(xmlDocumentHandleMerge2_);
}

TEST_F(DocumentHandleXMLFixture, checkMergeParams3) {  // NOLINT
  testMerge(xmlDocumentHandleMerge3_);
  testExtension(xmlDocumentHandleMerge3_);
}

TEST(DocumentHandleXML, badInclude) {  // NOLINT
  tinyxml_tools::DocumentHandleXML badIncludeHandle;
  ASSERT_FALSE(badIncludeHandle.create("xml_files/BadInclude.xml", tinyxml_tools::DocumentMode::READ));
}

TEST(DocumentHandleXML, badDefault) {  // NOLINT
  tinyxml_tools::DocumentHandleXML badDefaultHandle;
  ASSERT_FALSE(badDefaultHandle.create("xml_files/BadDefault.xml", tinyxml_tools::DocumentMode::READ));
}

TEST(DocumentHandleXML, badExtension) {  // NOLINT
  tinyxml_tools::DocumentHandleXML badExtensionHandle;
  ASSERT_FALSE(badExtensionHandle.create("xml_files/BadExtension.xml", tinyxml_tools::DocumentMode::READ));
}

TEST(DocumentHandleXML, badStructure) {  // NOLINT
  tinyxml_tools::DocumentHandleXML badStructureHandle;
  ASSERT_FALSE(badStructureHandle.create("xml_files/MissingStructureParams.xml", tinyxml_tools::DocumentMode::READ));
}
