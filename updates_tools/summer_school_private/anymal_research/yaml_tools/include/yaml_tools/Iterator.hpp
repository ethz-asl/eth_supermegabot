#pragma once

// yaml tools
#include "yaml_tools/internal/IteratorBase.hpp"

namespace yaml_tools {

//! Iterator wrapper which updates the IteratorValue on dereferencing.
class Iterator : public internal::IteratorBase<Iterator> {
 public:
  //! Constructor.
  Iterator(const YamlNode* parentYamlNode, bool begin) : internal::IteratorBase<Iterator>(parentYamlNode, begin) {}

  //! Dereference IteratorValue by star operator.
  IteratorValue& operator*() {
    // Update the IteratorValue only on dereferencing.
    // Updating it anywhere else can lead to segmentation faults in case it points to an invalid location.
    update();
    return iteratorValue_;
  }

  //! Dereference IteratorValue py pointer operator.
  const IteratorValue* operator->() {
    // Update the IteratorValue only on dereferencing.
    // Updating it anywhere else can lead to segmentation faults in case it points to an invalid location.
    update();
    return &iteratorValue_;
  }
};

}  // namespace yaml_tools
