#pragma once

// std
#include <string>

// yaml-cpp
#include <yaml-cpp/yaml.h>

// yaml tools
#include <yaml_tools/YamlNode.hpp>

namespace yaml_tools {
namespace internal {

/*!
 * yaml-cpp does not sort maps while emitting them (used for writing to strings).
 * This helper function solves the issue by sorting the map by its keys before emitting it.
 * The implementation has been taken from a comment in the following issue report:
 * https://github.com/jbeder/yaml-cpp/issues/169
 */

// Recursive helper function that does all the work.
void nodeToOrderedEmitter(const YamlNode& node, YAML::Emitter& emitter);

// Main function that emits a YAML node to an output stream.
std::string nodeToOrderedString(const YamlNode& node);

}  // namespace internal
}  // namespace yaml_tools
