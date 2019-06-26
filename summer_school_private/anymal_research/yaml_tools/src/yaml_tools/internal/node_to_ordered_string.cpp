// std
#include <algorithm>
#include <string>
#include <vector>

// yaml-cpp
#include <yaml-cpp/yaml.h>

// yaml tools
#include "yaml_tools/ConstIterator.hpp"
#include "yaml_tools/Iterator.hpp"
#include "yaml_tools/YamlNode.hpp"
#include "yaml_tools/internal/node_to_ordered_string.hpp"

namespace yaml_tools {
namespace internal {

void nodeToOrderedEmitter(const YamlNode& yamlNode, YAML::Emitter& emitter) {
  switch (yamlNode.getType()) {
    case YamlNode::Type::Sequence: {
      emitter << YAML::BeginSeq;
      for (const auto& subNode : yamlNode) {
        nodeToOrderedEmitter(subNode, emitter);
      }
      emitter << YAML::EndSeq;
      break;
    }
    case YamlNode::Type::Map: {
      emitter << YAML::BeginMap;
      for (const auto& key : yamlNode.getKeys()) {
        emitter << YAML::Key;
        emitter << key;
        emitter << YAML::Value;
        nodeToOrderedEmitter(yamlNode[key], emitter);
      }
      emitter << YAML::EndMap;
      break;
    }
    default:
      emitter << yamlNode.as<std::string>();
      break;
  }
}

// Main function that emits a yaml node to an output stream.
std::string nodeToOrderedString(const YamlNode& node) {
  YAML::Emitter emitter;
  nodeToOrderedEmitter(node, emitter);
  return std::string(emitter.c_str());
}

}  // namespace internal
}  // namespace yaml_tools
