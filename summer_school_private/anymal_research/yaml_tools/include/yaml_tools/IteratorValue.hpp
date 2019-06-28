#pragma once

// std
#include <string>
#include <tuple>

// yaml tools
#include "yaml_tools/YamlNode.hpp"

namespace yaml_tools {

//! The iterator value can hold a sequence (YAML node) or map iterator (key + YAML node) object.
class IteratorValue : public YamlNode, public std::pair<std::string, YamlNode> {};

}  // namespace yaml_tools
