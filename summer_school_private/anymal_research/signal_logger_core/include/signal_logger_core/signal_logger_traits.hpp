/*!
 * @file     signal_logger_traits.hpp
 * @author   Gabriel Hottiger, Christian Gehring
 * @date     Sep 28, 2016
 * @brief    Traits for the supported non-STL types.
 */

#pragma once

// signal logger
#include "signal_logger_core/LogElementTypes.hpp"

// kindr
#ifdef SILO_USE_KINDR
#include <kindr/Core>
#endif

// Eigen
#include <Eigen/Core>

// STL
#include <type_traits>
#include <string>
#include <array>
#include <map>
#include <unordered_map>

namespace signal_logger {

namespace traits {

template<typename T>
using element_type_t = typename std::remove_cv<typename std::remove_reference<decltype(*std::begin(std::declval<T&>()))>::type>::type;

//----------------------------------- STL traits -------------------------------------//

template <typename>
struct is_std_array : std::false_type {};

template <typename V, size_t n>
struct is_std_array<std::array<V, n>> : std::true_type {};

//! is_map false type
template<typename>
struct is_map : std::false_type {};

//! is_map true type
template<typename T, typename U>
struct is_map<std::map<T,U>> : std::true_type {};

//! is_map true type
template<typename T, typename U>
struct is_map<std::unordered_map<T,U>> : std::true_type {};

//! is_pair false type
template<typename>
struct is_pair : std::false_type {};

//! is_pair true type
template<typename T, typename U>
struct is_pair<std::pair<T,U>> : std::true_type {};

//! is_pair false type
template<typename Pair_, typename FirstType_, typename SecondType_, typename Enable_ = void>
struct is_pair_of : std::false_type {};

//! is_pair true type
template<typename Pair_, typename FirstType_, typename SecondType_>
struct is_pair_of<Pair_, FirstType_, SecondType_, typename std::enable_if<is_pair<Pair_>::value &&
    std::is_same<typename Pair_::first_type, FirstType_>::value &&
    std::is_same<typename Pair_::second_type, SecondType_>::value>::type> : std::true_type {};

template<typename T>
struct has_const_iterator
{
 private:
  typedef char                      one;
  typedef struct { char array[2]; } two;

  template<typename C> static one test(typename C::const_iterator*);
  template<typename C> static two  test(...);
 public:
  static const bool value = sizeof(test<T>(0)) == sizeof(one);
  typedef T type;
};

template <typename T>
struct has_begin_end
{
  struct Dummy { typedef void const_iterator; };
  typedef typename std::conditional<has_const_iterator<T>::value, T, Dummy>::type TType;
  typedef typename TType::const_iterator iter;

  struct Fallback { iter begin() const; iter end() const; };
  struct Derived : TType, Fallback { };

  template<typename C, C> struct ChT;

  template<typename C> static char (&f(ChT<iter (Fallback::*)() const, &C::begin>*))[1];
  template<typename C> static char (&f(...))[2];
  template<typename C> static char (&g(ChT<iter (Fallback::*)() const, &C::end>*))[1];
  template<typename C> static char (&g(...))[2];

  static bool const beg_value = sizeof(f<Derived>(0)) == 2;
  static bool const end_value = sizeof(g<Derived>(0)) == 2;
};

template <typename T>
struct is_container
{
  static const bool value = has_const_iterator<T>::value && !std::is_same<std::string, typename std::remove_cv<T>::type>::value &&
		  has_begin_end<T>::beg_value && has_begin_end<T>::end_value;
};

//----------------------------------- EIGEN traits -------------------------------------//

//! isEigenQuaternion false type
template<typename>
struct is_eigen_quaternion : std::false_type {};

//! isEigenQuaternion true type
template<typename PrimType_, int Options_>
struct is_eigen_quaternion<Eigen::Quaternion<PrimType_, Options_>> : std::true_type {};

//! isEigenAngleAxis false type
template<typename>
struct is_eigen_angle_axis : std::false_type {};

//! isEigenAngleAxis true type
template<typename PrimType_>
struct is_eigen_angle_axis<Eigen::AngleAxis<PrimType_>> : std::true_type {};

//! isEigenVector3 false type
template<typename>
struct is_eigen_vector3 : std::false_type {};

//! isEigenVector3 true type
template<typename PrimType_>
struct is_eigen_vector3<Eigen::Matrix<PrimType_, 3, 1>> : std::true_type {};

//! isEigenMatrix false type
template<typename ValueType_, typename Enable_ = void>
struct is_eigen_matrix : std::false_type {};

//! isEigenMatrix true type
template<typename ValueType_>
struct is_eigen_matrix<ValueType_, typename std::enable_if< std::is_base_of< Eigen::MatrixBase<ValueType_>,
                                                            ValueType_ >::value>::type > : std::true_type {};

//! isEigenOfScalarMartix false type
template<typename ValueType_, typename PrimType_, typename Enable_ = void>
struct is_eigen_matrix_of_scalar : std::false_type {};

//! isEigenOfScalarMartix true type
template<typename ValueType_, typename PrimType_>
struct is_eigen_matrix_of_scalar<ValueType_, PrimType_, typename std::enable_if< is_eigen_matrix<ValueType_>::value &&
		std::is_same< typename ValueType_::Scalar, PrimType_ >::value>::type> : std::true_type {};

//! isEigenMartix (All matrices/vectors except Eigen::Vector3 types) false type
template<typename ValueType_, typename Enable_ = void>
struct is_eigen_matrix_excluding_vector3 : std::false_type {};

//! isEigenMartix (All matrices/vectors except Eigen::Vector3 types) true type
template<typename ValueType_>
struct is_eigen_matrix_excluding_vector3<ValueType_, typename std::enable_if< is_eigen_matrix<ValueType_>::value
															&& !is_eigen_vector3<ValueType_>::value >::type > : std::true_type {};

//! isEigenOfScalarMartix (All matrices/vectors except Eigen::Vector3 types) false type
template<typename ValueType_, typename PrimType_, typename Enable_ = void>
struct is_eigen_matrix_of_scalar_excluding_vector3 : std::false_type {};

//! isEigenOfScalarMartix (All matrices/vectors except Eigen::Vector3 types) true type
template<typename ValueType_, typename PrimType_>
struct is_eigen_matrix_of_scalar_excluding_vector3<ValueType_, PrimType_, typename std::enable_if< is_eigen_matrix_excluding_vector3<ValueType_>::value &&
    std::is_same< typename ValueType_::Scalar, PrimType_ >::value>::type> : std::true_type {};

//-------------------------------------------------------------------------------------//


//----------------------------------- KINDR traits -------------------------------------//
#ifdef SILO_USE_KINDR

//! isKindrVector false type
template<typename>
struct is_kindr_vector : std::false_type {};

//! isKindrVector true type
template<enum kindr::PhysicalType PhysicalType_, typename PrimType_, int Dimension_>
struct is_kindr_vector<kindr::Vector<PhysicalType_,PrimType_, Dimension_>> : std::true_type {};

//! isKindrVectorAtPosition false type
template<typename>
struct is_kindr_vector_at_position : std::false_type {};

//! isKindrVectorAtPosition true type
template<enum kindr::PhysicalType PhysicalType_, typename PrimType_, int Dimension_>
struct is_kindr_vector_at_position<signal_logger::KindrVectorAtPosition<kindr::Vector<PhysicalType_,PrimType_, Dimension_>>> : std::true_type {};

//! isKindrHomogeneousTransformation false type
template<typename>
struct is_kindr_homogeneous_transformation : std::false_type {};

//! isKindrHomogeneousTransformation true type
template<typename PrimType_, typename Position_, typename Rotation_>
struct is_kindr_homogeneous_transformation<kindr::HomogeneousTransformation<PrimType_, Position_, Rotation_>> : std::true_type {};

#endif
//-------------------------------------------------------------------------------------//

} // end namespace traits

} // end namespace signal_logger
