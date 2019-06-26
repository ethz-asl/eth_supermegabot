/*
 * EigenExtensions.hpp
 *
 *  Created on: April 9, 2018
 *      Author: Yvain de Viragh
 */

#pragma once

// stl
#include <cmath>

// eigen
#include <Eigen/Core>
#include <Eigen/SparseCore>

// boost
#include <boost/math/special_functions/pow.hpp>


namespace robot_utils {

inline Eigen::VectorXi integerSequence(const int low, const int high) {
  return Eigen::VectorXi::LinSpaced(high - low + 1, low, high);
}

/*
 * In Matlab syntax this would be A(rowSubscripts,columnSubscripts) = B, where rowSubscripts and columnSubscripts are vectors of subscripts.
 */
template <typename DerivedA, typename DerivedB>
inline void setAtSubscripts(Eigen::DenseBase<DerivedA>& filledObject, const Eigen::DenseBase<DerivedB>& includedObject, const Eigen::VectorXi& rowSubscripts, const Eigen::VectorXi& columnSubscripts) {

  assert(includedObject.rows() == rowSubscripts.size());
  assert(includedObject.cols() == columnSubscripts.size());
  assert(rowSubscripts.minCoeff() >= 0);
  assert(columnSubscripts.minCoeff() >= 0);
  assert(rowSubscripts.maxCoeff() < filledObject.rows());
  assert(columnSubscripts.maxCoeff() < filledObject.cols());

  for(unsigned int i = 0u; i < rowSubscripts.size(); i++) {
    for(unsigned int j = 0u; j < columnSubscripts.size(); j++) {
      filledObject(rowSubscripts(i),columnSubscripts(j)) = includedObject(i,j);
    }
  }
}

/*
 * In Matlab syntax this would be A(rowSubscripts,columnSubscripts) = A(rowSubscripts,columnSubscripts) + B, where rowSubscripts and columnSubscripts are vectors of subscripts.
 */
template <typename DerivedA, typename DerivedB>
inline void addAtSubscripts(Eigen::DenseBase<DerivedA>& filledObject, const Eigen::DenseBase<DerivedB>& includedObject, const Eigen::VectorXi& rowSubscripts, const Eigen::VectorXi& columnSubscripts) {

  assert(includedObject.rows() == rowSubscripts.size());
  assert(includedObject.cols() == columnSubscripts.size());
  assert(rowSubscripts.minCoeff() >= 0);
  assert(columnSubscripts.minCoeff() >= 0);
  assert(rowSubscripts.maxCoeff() < filledObject.rows());
  assert(columnSubscripts.maxCoeff() < filledObject.cols());

  for(unsigned int i = 0u; i < rowSubscripts.size(); i++) {
    for(unsigned int j = 0u; j < columnSubscripts.size(); j++) {
      filledObject(rowSubscripts(i),columnSubscripts(j)) += includedObject(i,j);
    }
  }
}

/*
 * A function for building lists of triplets. Note: Duplicate entries are not identified, but simply appended.
 * In Matlab syntax this would be A(rowSubscripts,columnSubscripts) = A(rowSubscripts,columnSubscripts) + B, where rowSubscripts and columnSubscripts are vectors of subscripts.
 */
template <typename TypeA, typename DerivedB>
inline void addTripletsAtSubscripts(std::vector<Eigen::Triplet<TypeA>>& tripletList, const Eigen::DenseBase<DerivedB>& includedObject, const Eigen::VectorXi& rowSubscripts, const Eigen::VectorXi& columnSubscripts) {

  assert(includedObject.rows() == rowSubscripts.size());
  assert(includedObject.cols() == columnSubscripts.size());
  assert(rowSubscripts.minCoeff() >= 0);
  assert(columnSubscripts.minCoeff() >= 0);

  for(unsigned int i = 0u; i < rowSubscripts.size(); i++) {
    for(unsigned int j = 0u; j < columnSubscripts.size(); j++) {
      if(includedObject(i,j) == 0.0) continue;
      tripletList.emplace_back(Eigen::Triplet<TypeA>(rowSubscripts(i), columnSubscripts(j), includedObject(i,j)));
    }
  }
}

} // namespace robot_utils
