#pragma once

// std
#include <string>

namespace yaml_tools {
namespace internal {

bool numberIsInteger(const double number);

// Generic conversion trait.
template <typename Scalar>
std::string scalarToString(const Scalar& scalar) {
  std::stringstream ss;
  ss << scalar;
  return ss.str();
}

// Specialized conversion traits.
template <>
std::string scalarToString(const bool& scalar);

template <>
std::string scalarToString(const double& scalar);

template <>
std::string scalarToString(const std::string& scalar);

}  // namespace internal
}  // namespace yaml_tools
