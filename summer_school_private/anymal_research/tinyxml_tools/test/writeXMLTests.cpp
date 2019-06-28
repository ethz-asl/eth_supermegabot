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

struct WriteXMLFixture : public ::testing::Test {
  static void SetUpTestCase() {
    xmlDocument_ = std::unique_ptr<TiXmlDocument>(new TiXmlDocument());
    xmlRootHandle_ = new TiXmlElement("Root");
    xmlDocument_->LinkEndChild(new TiXmlDeclaration("1.0", "", ""));
    xmlDocument_->LinkEndChild(xmlRootHandle_.ToElement());
  }

  static void TearDownTestCase() { xmlDocument_->SaveFile("xml_files/Generated.xml"); }

  //! XML document (see xmlDocumentHandle_)
  static std::unique_ptr<TiXmlDocument> xmlDocument_;

  //! document handle to access the parameters
  static TiXmlHandle xmlRootHandle_;
};

std::unique_ptr<TiXmlDocument> WriteXMLFixture::xmlDocument_ = std::unique_ptr<TiXmlDocument>(nullptr);
TiXmlHandle WriteXMLFixture::xmlRootHandle_ = WriteXMLFixture::xmlDocument_.get();

TEST_F(WriteXMLFixture, writePrimitiveTypes) {  // NOLINT
  TiXmlHandle math(xmlRootHandle_);
  if (tinyxml_tools::createChildElement(math, xmlRootHandle_, "Math/Constants")) {
    int ten = 10;
    tinyxml_tools::writeParameter("", ten, math, "ten");
    double pi = 3.1415;
    tinyxml_tools::writeParameter("", pi, math, "pi");
  }
}

TEST_F(WriteXMLFixture, writeSTL) {  // NOLINT
  TiXmlHandle stl(xmlRootHandle_);
  if (tinyxml_tools::createChildElement(stl, xmlRootHandle_, "STL")) {
    std::vector<std::string> vector{"ap", "ba", "ci"};
    tinyxml_tools::writeParameter("Vector", vector, stl, {"Apple", "Banana", "Citron"});

    std::array<double, 4> array{{3.33, 6.66, 7.77, 8.88}};
    tinyxml_tools::writeParameter("Array", array, stl);

    std::pair<int, std::string> pair(5, "five");
    tinyxml_tools::writeParameter("Pair", pair, stl);
  }
}

TEST_F(WriteXMLFixture, writeEigen) {  // NOLINT
  Eigen::Vector3d vector3d(1.0, 2.0, 3.0);
  tinyxml_tools::writeParameter("Eigen/Vectors/XYZ", vector3d, xmlRootHandle_, {"x", "y", "z"});

  Eigen::VectorXf vector2Dynamic(2);
  vector2Dynamic << 2.71, 3.14;
  tinyxml_tools::writeParameter("Eigen/Vectors/Dynamic", vector2Dynamic, xmlRootHandle_);

  Eigen::Matrix3d matrix3Fixed;
  matrix3Fixed << 1.0, 2.3, 4.6,  // clang-format off
                  5.2, 1.9, 1.2,
                  2.7, 6.3, 4.3;  // clang-format on
  tinyxml_tools::writeParameter("Eigen/Matrices/Fixed", matrix3Fixed, xmlRootHandle_);

  Eigen::MatrixXi matrix22Dynamic(2, 2);
  matrix22Dynamic << 15, 30,  // clang-format off
                     45, 60;  // clang-format on
  tinyxml_tools::writeParameter("Eigen/Matrices/Dynamic", matrix22Dynamic, xmlRootHandle_, {"v15", "v30", "v45", "v60"});
}

TEST_F(WriteXMLFixture, writeSiblings) {  // NOLINT
  TiXmlHandle siblings(xmlRootHandle_);

  if (tinyxml_tools::createChildElement(siblings, xmlRootHandle_, "Siblings")) {
    std::vector<float> vecFloat{1.5, 3.6, 4.7, 5.6};
    tinyxml_tools::writeParameterSiblings("Vector/vecFloat", vecFloat, siblings, "entry");

    std::array<char, 5> arrChar{'a', 'r', 'r', 'a', 'y'};
    tinyxml_tools::writeParameterSiblings("Array/arrChar", arrChar, siblings, "char");

    std::map<std::string, int> stringIntMap;
    stringIntMap.insert(std::make_pair("hello", 5));
    stringIntMap.insert(std::make_pair("world", 5));
    stringIntMap.insert(std::make_pair("!", 1));
    tinyxml_tools::writeParameterSiblings("Map/kv", stringIntMap, siblings, std::make_pair("key", "value"));

    std::unordered_map<std::string, std::string> unordered_map;
    unordered_map.insert(std::make_pair("Un", "or"));
    unordered_map.insert(std::make_pair("de", "re"));
    unordered_map.insert(std::make_pair("dM", "ap"));
    tinyxml_tools::writeParameterSiblings("Unorderd_Map/list", unordered_map, siblings);
  }
}
