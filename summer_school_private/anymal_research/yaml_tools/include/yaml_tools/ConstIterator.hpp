#pragma once

// yaml tools
#include "yaml_tools/internal/IteratorBase.hpp"

namespace yaml_tools {

//! Const iterator wrapper which updates the IteratorValue on dereferencing.
class ConstIterator : public internal::IteratorBase<ConstIterator> {
 public:
  //! Constructor.
  ConstIterator(const YamlNode* parentYamlNode, bool begin) : internal::IteratorBase<ConstIterator>(parentYamlNode, begin) {}

  //! Dereference IteratorValue by star operator.
  const IteratorValue& operator*() const {
    // Update the IteratorValue only on dereferencing.
    // Updating it anywhere else can lead to segmentation faults in case it points to an invalid location.
    update();
    return iteratorValue_;
  }

  //! Dereference IteratorValue py pointer operator.
  const IteratorValue* operator->() const {
    // Update the IteratorValue only on dereferencing.
    // Updating it anywhere else can lead to segmentation faults in case it points to an invalid location.
    update();
    return &iteratorValue_;
  }
};

}  // namespace yaml_tools
