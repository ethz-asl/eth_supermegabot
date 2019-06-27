#pragma once

// yaml tools
#include "yaml_tools/IteratorValue.hpp"
#include "yaml_tools/YamlNode.hpp"

namespace yaml_tools {
namespace internal {

//! Iterator base class.
template <typename T>
class IteratorBase {
 protected:
  const YamlNode* parentYamlNode_ = nullptr;
  std::vector<std::string> keys_;
  size_t index_ = 0;

  //! Value the iterator points to.
  //! Mutable to dereference it in in constant accessors.
  mutable IteratorValue iteratorValue_;

 protected:
  //! Constructor.
  IteratorBase(const YamlNode* parentYamlNode, bool begin) : parentYamlNode_(parentYamlNode) {
    if (parentYamlNode_->getType() != YamlNode::Type::Sequence && parentYamlNode_->getType() != YamlNode::Type::Map) {
      throw Exception("Node type is not iterable.");
    }
    if (parentYamlNode_->getType() == YamlNode::Type::Map) {
      keys_ = parentYamlNode_->getKeys();
    }
    if (begin) {
      index_ = 0;
    } else {
      index_ = parentYamlNode_->size();
    }
  }

 public:
  //! Comparison operators.
  bool operator==(const IteratorBase<T>& other) const { return parentYamlNode_ == other.parentYamlNode_ && index_ == other.index_; }
  bool operator!=(const IteratorBase<T>& other) const { return !(*this == other); }

  //! Prefix increment.
  T& operator++() {
    index_++;
    return static_cast<T&>(*this);
  }

  //! Postfix increment.
  T operator++(int) {
    const T& iterator = static_cast<const T&>(*this);
    ++(*this);
    return iterator;
  }

  //! Update the iterator value.
  void update() const {
    try {
      switch (parentYamlNode_->getType()) {
        case YamlNode::Type::Sequence:
          static_cast<YamlNode&>(iteratorValue_) = (*parentYamlNode_)[index_];
          break;
        case YamlNode::Type::Map:
          iteratorValue_.first = keys_[index_];
          iteratorValue_.second = (*parentYamlNode_)[iteratorValue_.first];
          break;
        default:
          throw Exception("Node type is not iterable.");
          break;
      }
    } catch (const YAML::Exception& exception) {
      throw Exception(std::string("Updating iterator failed: ") + std::string(exception.what()));
    }
  }
};

}  // namespace internal
}  // namespace yaml_tools
