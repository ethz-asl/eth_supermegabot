/*!
 * @file     yaml_helper.hpp
 * @author   Hersh(https://github.com/hersh), Gabriel Hottiger
 * @date     Oct 3, 2016
 * @brief    Ordered yaml write. Resolves open issue on https://github.com/jbeder/yaml-cpp/issues/169.
 */
//! The following code is also taken from there!

#pragma once

#include <algorithm>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace yaml_helper {

// Recursive helper function that does all the work
void writeNode(const YAML::Node& node, YAML::Emitter& emitter)
{
  switch (node.Type())
  {
    case YAML::NodeType::Sequence:
    {
      emitter << YAML::BeginSeq;
      for (size_t i = 0; i < node.size(); i++)
      {
        writeNode(node[i], emitter);
      }
      emitter << YAML::EndSeq;
      break;
    }
    case YAML::NodeType::Map:
    {
      emitter << YAML::BeginMap;

      // First collect all the keys
      std::vector<std::string> keys(node.size());
      int key_it = 0;
      for (YAML::const_iterator it = node.begin(); it != node.end(); ++it)
      {
        keys[key_it++] = it->first.as<std::string>();
      }

      // Then sort them
      std::sort(keys.begin(), keys.end());

      // Then emit all the entries in sorted order.
      for(std::vector<std::string>::const_iterator it = keys.end()-1; it >= keys.begin(); it--)
      {
        emitter << YAML::Key;
        emitter << *it;
        emitter << YAML::Value;
        writeNode(node[*it], emitter);
      }
      emitter << YAML::EndMap;
      break;
    }
    default:
      emitter << node;
      break;
  }
}

// Main function that emits a yaml node to an output stream.
void writeYamlOrderedMaps(std::ostream& out, const YAML::Node& node)
{
  YAML::Emitter emitter;
  writeNode(node, emitter);
  out << emitter.c_str() << std::endl;
}

}
