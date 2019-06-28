/*
 * EigenExtensionsTest.hpp
 *
 *  Created on: March 22, 2018
 *      Author: Yvain de Viragh
 */

#include <gtest/gtest.h>
#include <kindr/common/gtest_eigen.hpp>
#include <Eigen/Core>
#include <Eigen/SparseCore>
#include "robot_utils/eigen/EigenExtensions.hpp"
#include <vector>

/*
 * Test whether setAtSubcripts() and addAtSubcripts() work correctly for different objects.
 */

TEST(eigenTest, eigenExtensions_setAtSubscripts)
{
  using namespace robot_utils;

  Eigen::MatrixXi includedMatrix(2,3);
  Eigen::MatrixXd filledMatrix(3,4);
  Eigen::MatrixXd targetMatrix(3,4);
  Eigen::VectorXi rows(2);
  Eigen::VectorXi cols(3);

  includedMatrix << 1, 2, 3, 4, 5, 6;
  filledMatrix = -0.1*Eigen::MatrixXd::Ones(3,4);
  rows << 0, 2;
  cols << 0, 2, 3;
  targetMatrix << 1  , -0.1,  2  ,  3  ,
                 -0.1, -0.1, -0.1, -0.1,
                  4  , -0.1,  5  ,  6  ;

  setAtSubscripts(filledMatrix, includedMatrix, rows, cols);
  KINDR_ASSERT_DOUBLE_MX_EQ(filledMatrix, targetMatrix, 1.0e-10, "Fill a double matrix with an integer matrix");
};


TEST(eigenTest, eigenExtensions_addAtSubscripts)
{
  using namespace robot_utils;

  Eigen::VectorXi includedVector(3);
  Eigen::ArrayXd filledArray(6);
  Eigen::ArrayXd targetArray(6);
  Eigen::VectorXi rows(3);
  Eigen::VectorXi cols(1);

  includedVector = integerSequence(4,6);
  filledArray << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6;
  rows << 1, 3, 4;
  cols << 0;
  targetArray << 0.1, 4.2, 0.3, 5.4, 6.5, 0.6;

  addAtSubscripts(filledArray, includedVector, rows, cols);
  KINDR_ASSERT_DOUBLE_MX_EQ(filledArray, targetArray, 1.0e-10, "Add some elements of an integer vector to a double 1D array");

};


TEST(eigenTest, eigenExtensions_addTripletsAtSubscripts)
{
  using namespace robot_utils;

  Eigen::MatrixXi includedMatrix(2,3);
  std::vector<Eigen::Triplet<double>> tripletList;
  Eigen::SparseMatrix<double, Eigen::RowMajor> filledMatrix(3,4);
  Eigen::MatrixXd targetMatrix(3,4);
  Eigen::VectorXi rows(2);
  Eigen::VectorXi cols(3);

  includedMatrix << 1, 2, 3, 4, 5, 6;
  for(unsigned int i = 0u; i < 3u; i++) {
    for(unsigned int j = 0u; j < 4u; j++) {
      tripletList.emplace_back(Eigen::Triplet<double>(i, j, -0.1));
    }
  }
  rows << 0, 2;
  cols << 0, 2, 3;
  targetMatrix << 1, 0, 2, 3,
                  0, 0, 0, 0,
                  4, 0, 5, 6;
  targetMatrix.array() -= 0.1;

  addTripletsAtSubscripts(tripletList, includedMatrix, rows, cols);

  filledMatrix.setFromTriplets(tripletList.begin(), tripletList.end());
  KINDR_ASSERT_DOUBLE_MX_EQ(Eigen::MatrixXd(filledMatrix), Eigen::MatrixXd(targetMatrix), 1.0e-10,
                            "Fill a triplet list with an integer matrix and convert to sparse matrix.");
};

