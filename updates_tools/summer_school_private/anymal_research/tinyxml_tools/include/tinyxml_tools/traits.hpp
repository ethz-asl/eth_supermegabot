/*!
 * @file	  traits.hpp
 * @author	  Gabriel Hottiger
 * @date	  Jan, 2018
 */

#pragma once

// Eigen
#include <Eigen/Core>

// STL
#include <array>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

namespace tinyxml_tools {

namespace traits {

namespace map_helper {
//! is_map false type
template <typename>
struct is_map : std::false_type {};

//! is_map true type
template <typename T, typename U>
struct is_map<std::map<T, U>> : std::true_type {};

//! is_map true type
template <typename T, typename U>
struct is_map<std::unordered_map<T, U>> : std::true_type {};
}  // namespace map_helper

//! Element trait for primitive types
template <typename T, typename Enable = void>
struct element {
  // Types
  using type = T;
  using stringType = std::string;

  // Default values
  static const stringType getDefaultName() { return stringType("value"); }
  static type getDefaultValue() { return type(); }
};

//! Element trait for string literals -> Fixes deduction errors
template <std::size_t N>
struct element<const char[N]> {
  // Types
  using type = const char*;
  using stringType = std::string;

  // Default values
  static const stringType getDefaultName() { return stringType("value"); }
  static type getDefaultValue() { return type(); }
};

//! Element trait for stl pair
template <typename T, typename U>
struct element<std::pair<T, U>> {
  // Types
  using type = std::pair<T, U>;
  using stringType = std::pair<std::string, std::string>;

  // Default values
  static const stringType getDefaultName() { return std::make_pair("first", "second"); }
  static type getDefaultValue() { return std::make_pair(T(), U()); }
};

//! Element trait for stl vectors
template <typename T>
struct element<std::vector<T>> {
  // Types
  using type = std::vector<T>;
  using stringType = std::vector<std::string>;
  using siblingStringType = std::string;
  using siblingType = T;

  // Default values
  static const stringType getDefaultName() { return stringType{}; }
  static type getDefaultValue() { return type{}; }
};

//! Element trait for stl arrays
template <typename T, std::size_t N>
struct element<std::array<T, N>> {
  // Types
  using type = std::array<T, N>;
  using stringType = std::array<std::string, N>;
  using siblingStringType = std::string;
  using siblingType = T;

  // Default values
  static const stringType getDefaultName() {
    stringType names;
    for (size_t i = 0; i < N; ++i) {
      names[i] = std::string("a_") + std::to_string(i);
    }
    return names;
  }
  static type getDefaultValue() { return type{}; }
};

//! Element trait for stl maps
template <typename T>
struct element<T, typename std::enable_if<map_helper::is_map<T>::value>::type> {
  // Types
  using type = T;
  using siblingType = std::pair<typename T::key_type, typename T::mapped_type>;
  using siblingStringType = typename element<siblingType>::stringType;
};

template <typename T>
struct element<T, typename std::enable_if<std::is_base_of<typename Eigen::MatrixBase<T>, T>::value>::type> {
  // Types
  using type = T;
  using stringType = std::vector<std::string>;

  // Default values
  static const stringType getDefaultName() {
    stringType names;
    for (int i = 0; i < T::RowsAtCompileTime; ++i) {
      for (int j = 0; j < T::ColsAtCompileTime; ++j) {
        names.push_back(std::string("m_") + std::to_string(i) + std::to_string(j));
      }
    }
    return names;
  }
  static type getDefaultValue() {
    return type::Zero(std::max(0, static_cast<int>(T::RowsAtCompileTime)), std::max(0, static_cast<int>(T::ColsAtCompileTime)));
  }
};

}  // namespace traits

}  // namespace tinyxml_tools
