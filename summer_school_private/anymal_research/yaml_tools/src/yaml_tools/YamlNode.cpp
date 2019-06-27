// yaml tools
#include "yaml_tools/YamlNode.hpp"
#include "yaml_tools/ConstIterator.hpp"
#include "yaml_tools/Iterator.hpp"
#include "yaml_tools/internal/node_to_ordered_string.hpp"

namespace yaml_tools {

YamlNode::YamlNode(const YAML::Node& impl, std::string nestedKey) : impl_(impl), nestedKey_(std::move(nestedKey)) {}

YamlNode::Impl& YamlNode::getImpl() { return impl_; }

const YamlNode::Impl& YamlNode::getImpl() const { return impl_; }

YamlNode YamlNode::fromString(const std::string& string) {
  try {
    return YamlNode(YAML::Load(string), std::string());
  } catch (const YAML::Exception& exception) {
    throw Exception(std::string("Reading YAML node from string failed: ") + std::string(exception.what()));
  }
}

std::string YamlNode::toString() const {
  try {
    if (impl_.IsNull()) {
      return std::string();
    }
    return internal::nodeToOrderedString(*this);
  } catch (const YAML::Exception& exception) {
    throw Exception(std::string("Writing YAML node to string failed: ") + std::string(exception.what()));
  }
}

YamlNode YamlNode::fromFile(const std::string& path) {
  try {
    return YamlNode(YAML::LoadFile(path), path);
  } catch (const YAML::Exception& exception) {
    throw Exception(std::string("Reading YAML node from file '" + path + "' failed: ") + std::string(exception.what()));
  }
}

void YamlNode::toFile(const std::string& path) const {
  std::ofstream file;
  file.open(path.c_str());
  if (!file.is_open()) {
    throw Exception(std::string("Writing YAML node to file '" + path + "' failed (File could not be opened)."));
  }
  file << toString();
  file << std::endl << std::endl;
  file.close();
}

bool YamlNode::operator==(const YamlNode& rhs) const {
  // Immediately return true for self comparison.
  if (this == &rhs) {
    return true;
  }

  // Compare nodes after conversion to string.
  return toString() == rhs.toString();
}

bool YamlNode::hasKey(const std::string& key) const {
  const std::string nestedKey = nestedKey_ + "/" + key;
  try {
    return static_cast<bool>(impl_[key]);
  } catch (const YAML::Exception& exception) {
    throw Exception(std::string("Checking for key '" + nestedKey + "' failed: ") + std::string(exception.what()));
  }
}

bool YamlNode::hasKey(const size_t i) const {
  const std::string nestedKey = nestedKey_ + "/" + std::to_string(i);
  try {
    return static_cast<bool>(impl_[i]);
  } catch (const YAML::Exception& exception) {
    throw Exception(std::string("Checking for key '" + nestedKey + "' failed: ") + std::string(exception.what()));
  }
}

std::vector<std::string> YamlNode::getKeys() const {
  // Check if the node is a map.
  if (getType() != YamlNode::Type::Map) {
    throw Exception(std::string("The node must be of type map to get the keys."));
  }
  std::vector<std::string> keys;
  // First collect all the keys.
  try {
    for (auto it = impl_.begin(); it != impl_.end(); ++it) {
      keys.push_back(it->first.as<std::string>());
    }
  } catch (const YAML::Exception& exception) {
    throw Exception(std::string("Getting keys failed (are your keys of type string?): ") + std::string(exception.what()));
  }
  // Then sort them alphabetically.
  std::sort(keys.begin(), keys.end());
  return keys;
}

YamlNode YamlNode::operator[](const std::string& key) {
  // In this non-const function, we do not need to check whether the key exists since
  // operator[..] will overwrite or create a new key.
  const std::string nestedKey = nestedKey_ + "/" + key;
  try {
    return YamlNode(impl_[key], nestedKey);
  } catch (const YAML::Exception& exception) {
    throw Exception(std::string("Accessing node at '" + nestedKey + "' failed: ") + std::string(exception.what()));
  }
}

YamlNode YamlNode::operator[](const std::string& key) const {
  // In this const function, overwriting or creating a new key is not allowed.
  const std::string nestedKey = nestedKey_ + "/" + key;
  if (!hasKey(key)) {
    throw Exception(std::string("Accessing node at '" + nestedKey + "' failed: The key does not exist."));
  }
  try {
    return YamlNode(impl_[key], nestedKey);
  } catch (const YAML::Exception& exception) {
    throw Exception(std::string("Accessing node at '" + nestedKey + "' failed: ") + std::string(exception.what()));
  }
}

YamlNode YamlNode::operator[](const size_t i) {
  const std::string nestedKey = nestedKey_ + "/" + std::to_string(i);
  if (!hasKey(i)) {
    throw Exception(std::string("Accessing node at '" + nestedKey + "' failed: The key does not exist."));
  }
  try {
    return YamlNode(impl_[i], nestedKey);
  } catch (const YAML::Exception& exception) {
    throw Exception(std::string("Accessing node at '" + nestedKey + "' failed: ") + std::string(exception.what()));
  }
}

YamlNode YamlNode::operator[](const size_t i) const {
  const std::string nestedKey = nestedKey_ + "/" + std::to_string(i);
  if (!hasKey(i)) {
    throw Exception(std::string("Accessing node at '" + nestedKey + "' failed: The key does not exist."));
  }
  try {
    return YamlNode(impl_[i], nestedKey);
  } catch (const YAML::Exception& exception) {
    throw Exception(std::string("Accessing node at '" + nestedKey + "' failed: ") + std::string(exception.what()));
  }
}

size_t YamlNode::size() const {
  try {
    return impl_.size();
  } catch (const YAML::Exception& exception) {
    throw Exception(std::string("Reading the size of '" + nestedKey_ + "' failed: ") + std::string(exception.what()));
  }
}

Iterator YamlNode::begin() { return Iterator(this, true); }

ConstIterator YamlNode::begin() const { return ConstIterator(this, true); }

Iterator YamlNode::end() { return Iterator(this, false); }

ConstIterator YamlNode::end() const { return ConstIterator(this, false); }

template <>
void YamlNode::pushBack(const YamlNode& t) {
  pushBack(t.getImpl());
}

}  // namespace yaml_tools
