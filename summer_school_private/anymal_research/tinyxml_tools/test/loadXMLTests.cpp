/*!
 * @file     loadXMLTests.cpp
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

struct LoadXMLFixture : public ::testing::Test {
  LoadXMLFixture() : xmlDocument_(), xmlDocumentHandle_(&xmlDocument_) {
    if (!xmlDocument_.LoadFile("xml_files/Generated.xml")) {
      throw std::exception();
    }
  }
  //! XML document (see xmlDocumentHandle_)
  TiXmlDocument xmlDocument_;

  //! document handle to access the parameters
  TiXmlHandle xmlDocumentHandle_;
};

TEST_F(LoadXMLFixture, loadPrimitiveTypes) {  // NOLINT
  // Get handle
  TiXmlHandle mathConstantsHandle(xmlDocumentHandle_);
  ASSERT_TRUE(tinyxml_tools::getChildHandle(mathConstantsHandle, xmlDocumentHandle_, "Root/Math/Constants"));

  // Get double
  double pi = 0.0;
  ASSERT_TRUE(tinyxml_tools::loadParameter(pi, mathConstantsHandle, "pi", 0.0));
  ASSERT_EQ(pi, 3.1415);

  // Get int
  int ten = 0;
  ASSERT_TRUE(tinyxml_tools::loadParameter(ten, mathConstantsHandle, "ten", 0));
  ASSERT_EQ(ten, 10);
}

TEST_F(LoadXMLFixture, loadSTLContainers) {  // NOLINT
  // Get handle
  TiXmlHandle stlHandle(xmlDocumentHandle_);
  ASSERT_TRUE(tinyxml_tools::getChildHandle(stlHandle, xmlDocumentHandle_, "Root/STL"));

  // Get std vector
  std::vector<std::string> fruitsVector(3);
  ASSERT_TRUE(tinyxml_tools::loadParameter("Vector", fruitsVector, stlHandle, {"Apple", "Banana", "Citron"}));
  ASSERT_EQ(fruitsVector[0], "ap");
  ASSERT_EQ(fruitsVector[1], "ba");
  ASSERT_EQ(fruitsVector[2], "ci");

  // Get std array
  std::array<double, 4> doubleArray{};
  ASSERT_TRUE(tinyxml_tools::loadParameter("Array", doubleArray, stlHandle));
  ASSERT_EQ(doubleArray[0], 3.33);
  ASSERT_EQ(doubleArray[1], 6.66);
  ASSERT_EQ(doubleArray[2], 7.77);
  ASSERT_EQ(doubleArray[3], 8.88);

  // Get std array
  std::pair<int, std::string> intStringPair{};
  ASSERT_TRUE(tinyxml_tools::loadParameter("Pair", intStringPair, stlHandle));
  ASSERT_EQ(intStringPair.first, 5);
  ASSERT_EQ(intStringPair.second, "five");
}

TEST_F(LoadXMLFixture, loadEigen) {  // NOLINT
  // Get eigen vector
  Eigen::Vector3d xyzVector;
  Eigen::Vector3d xyzVectorDes(1.0, 2.0, 3.0);
  ASSERT_TRUE(tinyxml_tools::loadParameter("Root/Eigen/Vectors/XYZ", xyzVector, xmlDocumentHandle_, {"x", "y", "z"}));
  ASSERT_TRUE(xyzVector.isApprox(xyzVectorDes));

  // Get eigen vector
  Eigen::VectorXf xfVector(2);
  Eigen::VectorXf xfVectorDes(2);
  xfVectorDes << 2.71, 3.14;
  ASSERT_TRUE(tinyxml_tools::loadParameter("Root/Eigen/Vectors/Dynamic", xfVector, xmlDocumentHandle_));
  ASSERT_TRUE(xfVector.isApprox(xfVectorDes));

  Eigen::Matrix3d matrix3Fixed;
  Eigen::Matrix3d matrix3FixedDes;
  matrix3FixedDes << 1.0, 2.3, 4.6,  // clang-format off
                     5.2, 1.9, 1.2,
                     2.7, 6.3, 4.3;  // clang-format on
  ASSERT_TRUE(tinyxml_tools::loadParameter("Root/Eigen/Matrices/Fixed", matrix3Fixed, xmlDocumentHandle_));
  ASSERT_TRUE(matrix3Fixed.isApprox(matrix3FixedDes));

  Eigen::MatrixXi matrix22Dynamic(2, 2);
  Eigen::MatrixXi matrix22DynamicDes(2, 2);
  matrix22DynamicDes << 15, 30,  // clang-format off
                        45, 60;  // clang-format on
  ASSERT_TRUE(
      tinyxml_tools::loadParameter("Root/Eigen/Matrices/Dynamic", matrix22Dynamic, xmlDocumentHandle_, {"v15", "v30", "v45", "v60"}));
  ASSERT_TRUE(matrix22Dynamic.isApprox(matrix22DynamicDes));
}

TEST_F(LoadXMLFixture, loadSiblings) {  // NOLINT
  // Get handle
  TiXmlHandle siblingsHandle(xmlDocumentHandle_);
  ASSERT_TRUE(tinyxml_tools::getChildHandle(siblingsHandle, xmlDocumentHandle_, "Root/Siblings"));

  // Get stl vector
  std::vector<float> vector;
  ASSERT_TRUE(tinyxml_tools::loadParameterSiblings("Vector/vecFloat", vector, siblingsHandle, "entry"));
  ASSERT_EQ(vector.at(0), 1.5f);
  ASSERT_EQ(vector.at(1), 3.6f);
  ASSERT_EQ(vector.at(2), 4.7f);
  ASSERT_EQ(vector.at(3), 5.6f);

  // Get stl vector
  std::array<char, 5> array{};
  ASSERT_TRUE(tinyxml_tools::loadParameterSiblings("Array/arrChar", array, siblingsHandle, "char"));
  ASSERT_EQ(array[0], 'a');
  ASSERT_EQ(array[1], 'r');
  ASSERT_EQ(array[2], 'r');
  ASSERT_EQ(array[3], 'a');
  ASSERT_EQ(array[4], 'y');

  // Get std map
  std::map<std::string, int> stringIntMap{};
  ASSERT_TRUE(tinyxml_tools::loadParameterSiblings("Map/kv", stringIntMap, siblingsHandle, std::make_pair("key", "value")));
  ASSERT_EQ(stringIntMap.at("hello"), 5);
  ASSERT_EQ(stringIntMap.at("world"), 5);
  ASSERT_EQ(stringIntMap.at("!"), 1);

  // Get std unordered map
  std::unordered_map<std::string, std::string> unordered_map{};
  ASSERT_TRUE(tinyxml_tools::loadParameterSiblings("Unorderd_Map/list", unordered_map, siblingsHandle));
  ASSERT_EQ(unordered_map.at("Un"), "or");
  ASSERT_EQ(unordered_map.at("de"), "re");
  ASSERT_EQ(unordered_map.at("dM"), "ap");
}
