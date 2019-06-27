// std
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>

// yaml tools
#include "yaml_tools/internal/scalar_to_string.hpp"

namespace yaml_tools {
namespace internal {

template <>
std::string scalarToString(const bool& scalar) {
  // Use names for booleans.
  std::stringstream ss;
  ss << std::boolalpha << scalar;
  return ss.str();
}

template <>
std::string scalarToString(const double& scalar) {
  if (std::isnan(scalar)) {
    return std::string(".nan");
  } else if (std::isinf(scalar)) {
    if (scalar > 0.0) {
      return std::string(".inf");
    } else {
      return std::string("-.inf");
    }
  } else if (scalar == std::floor(scalar)) {
    // For whole numbers, make sure there is a decimal point.
    std::stringstream ss;
    ss << scalar;
    std::string string = ss.str();
    // Whole numbers in scientific notation already contain a decimal point.
    if (string.find('.') == std::string::npos) {
      string += std::string(".0");
    }
    return string;
  } else {
    // For non-whole numbers, use maximal precision.
    std::stringstream ss;
    ss << std::setprecision(std::numeric_limits<double>::digits10) << scalar;
    return ss.str();
  }
}

template <>
std::string scalarToString(const std::string& scalar) {
  return scalar;
}

}  // namespace internal

}  // namespace yaml_tools
