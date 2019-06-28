#pragma once

// std
#include <cstdint>
#include <fstream>
#include <iostream>

// yaml-cpp
#include <yaml-cpp/yaml.h>

// yaml tools
#include "yaml_tools/Exception.hpp"
#include "yaml_tools/internal/scalar_to_string.hpp"

namespace yaml_tools {

//! Forward declarations.
class IteratorValue;
class Iterator;
class ConstIterator;

/*!
 * YAML node wrapping class.
 * YAML nodes can contain trees of the following data structures:
 * - scalars (of type bool, int, float, double, string)
 * - sequences
 * - maps
 */
class YamlNode {
 public:
  //! The underlying node.
  using Impl = YAML::Node;
  //! The underlying node type.
  using Type = YAML::NodeType::value;

 protected:
  //! Implementation.
  Impl impl_;
  //! Nested key for printing warnings and errors.
  //! Contains information about file paths, sequence indices and map keys.
  std::string nestedKey_;

 protected:
  /*!
   * Constructor from an underlying YAML node.
   * Protected, for internal use only.
   * @param impl      Underlying YAML node.
   * @param nestedKey Nested key.
   */
  YamlNode(const Impl& impl, std::string nestedKey);

 public:
  //! Default constructor.
  YamlNode() = default;
  //! Default copy constructor (shallow!).
  YamlNode(const YamlNode& other) = default;
  //! Destructor.
  virtual ~YamlNode() = default;

  //! Get the underlying type as reference.
  Impl& getImpl();
  //! Get the underlying type as const reference.
  const Impl& getImpl() const;
  //! Get the nested key of the YAML node.
  const std::string& getNestedKey() const { return nestedKey_; }

  /*!
   * Create a new YAML node by parsing a string.
   * Throws a yaml_tools::Exception if unsuccessful.
   * @param string String to parse.
   * @return New YAML node.
   */
  static YamlNode fromString(const std::string& string);

  /*!
   * Convert the YAML node into a string.
   * Throws a yaml_tools::Exception if unsuccessful.
   * @return YAML node in string format.
   */
  std::string toString() const;

  /*!
   * Create a new YAML node by parsing a file.
   * Throws a yaml_tools::Exception if unsuccessful.
   * @param path Absolute path of the file.
   * @return New YAML node.
   */
  static YamlNode fromFile(const std::string& path);

  /*!
   * Write the YAML node into a file.
   * Throws a yaml_tools::Exception if unsuccessful.
   * @param path Absolute path of the file.
   */
  void toFile(const std::string& path) const;

  //! Check if the YAML node is defined.
  bool isDefined() const { return impl_.IsDefined(); }
  //! Check if the YAML node is null.
  bool isNull() const { return impl_.IsNull(); }
  //! Check if the YAML node is of type scalar.
  bool isScalar() const { return impl_.IsScalar(); }
  //! Check if the YAML node is of type sequence.
  bool isSequence() const { return impl_.IsSequence(); }
  //! Check if the YAML node is of type map.
  bool isMap() const { return impl_.IsMap(); }
  //! Get the YAML node type.
  Type getType() const { return impl_.Type(); }

  //! Assignment operator for YAML nodes (shallow!).
  YamlNode& operator=(const YamlNode& rhs) = default;

  /*!
   * Assignment operator for scalars.
   * @tparam Scalar Type of the scalar.
   * @param rhs Scalar to assign this YAML node to.
   * @return Reference to this YAML node.
   */
  template <typename Scalar>
  YamlNode& operator=(const Scalar& rhs) {
    // This is a workaround for the issue that yaml-cpp stores '1.0' as '1'.
    impl_ = internal::scalarToString(rhs);
    return *this;
  }

  /*!
   * Comparison operator.
   * Compares the content of the node, ignoring the nested keys.
   * @param rhs Other YAML node.
   * @return True if the YAML nodes are equal.
   */
  bool operator==(const YamlNode& rhs) const;

  /*!
   * Check if YAML node has a given key.
   * Throws a yaml_tools::Exception if unsuccessful.
   * @param key Key to check for.
   * @return True if contains given key.
   */
  bool hasKey(const std::string& key) const;

  /*!
   * Check if YAML node has a given index.
   * Throws a yaml_tools::Exception if unsuccessful.
   * @param i Index to check for.
   * @return True if contains given index.
   */
  bool hasKey(const size_t i) const;

  /*!
   * Get the map keys of this YAML node.
   * Applicable YAML node types: Map.
   * Throws a yaml_tools::Exception if unsuccessful.
   * @return Map keys of the YAML node.
   */
  std::vector<std::string> getKeys() const;

  /*!
   * Access a child YAML node by key.
   * Creates a new child YAML node if it is not existing yet.
   * Throws a yaml_tools::Exception if unsuccessful.
   * @param key Key.
   * @return YAML node.
   */
  YamlNode operator[](const std::string& key);

  /*!
   * Access a child YAML node by key.
   * Does not create a new child YAML node if it is not existing yet.
   * Throws a yaml_tools::Exception if unsuccessful.
   * @param key Key.
   * @return YAML node.
   */
  YamlNode operator[](const std::string& key) const;

  /*!
   * Access a child YAML node by index.
   * Does not create a new child YAML node if it is not existing yet.
   * Throws a yaml_tools::Exception if unsuccessful.
   * Note: This method does the same thing as its 'const' counterpart.
   *       It has been introduced to solve the nonConstYamlNode[0] ambiguity
   *       issue, which origins from the fact that '0' can be converted to
   *       size_t, but also to 'const char*' and therefore 'string'.
   * @param i Index.
   * @return YAML node.
   */
  YamlNode operator[](const size_t i);

  /*!
   * Access a child YAML node by index.
   * Does not create a new child YAML node if it is not existing yet.
   * Throws a yaml_tools::Exception if unsuccessful.
   * @param i Index.
   * @return YAML node.
   */
  YamlNode operator[](const size_t i) const;

  /*!
   * Get the number of elements contained in this YAML node.
   * Returns 0 for YAML nodes of type Scalar.
   * Applicable YAML node types: Scalar, Sequence, Map.
   * Throws a yaml_tools::Exception if unsuccessful.
   * @return Size of the YAML node.
   */
  size_t size() const;

  /*!
   * Add an element to the YAML node.
   * Applicable YAML node types: Sequence.
   * Throws a yaml_tools::Exception if unsuccessful.
   * @tparam T Type of the element. Can be a scalar or a YAML node itself.
   * @param t Element to add.
   */
  template <typename T>
  void pushBack(const T& t) {
    try {
      impl_.push_back(t);
    } catch (const YAML::Exception& exception) {
      throw Exception(std::string("Pushing back an element failed: ") + std::string(exception.what()));
    }
  }

  /*!
   * Get an iterator pointing to the first element of the YAML node.
   * Applicable YAML node types: Sequence, Map.
   * Throws a yaml_tools::Exception if unsuccessful.
   * @return Iterator.
   */
  Iterator begin();

  /*!
   * Get a constant iterator pointing to the first element of the YAML node.
   * Applicable YAML node types: Sequence, Map.
   * Throws a yaml_tools::Exception if unsuccessful.
   * @return Constant iterator.
   */
  ConstIterator begin() const;

  /*!
   * Get an iterator pointing to one past the last element of the YAML node.
   * Applicable YAML node types: Sequence, Map.
   * Throws a yaml_tools::Exception if unsuccessful.
   * @return Iterator.
   */
  Iterator end();

  /*!
   * Get a constant iterator pointing to one past the last element of the YAML node.
   * Applicable YAML node types: Sequence, Map.
   * Throws a yaml_tools::Exception if unsuccessful.
   * @return Constant iterator.
   */
  ConstIterator end() const;

  /*!
   * Cast a YAML node to a scalar.
   * Applicable YAML node types: Scalar.
   * Throws a yaml_tools::Exception if unsuccessful.
   * @tparam Scalar Scalar type to cast into.
   * @return Scalar.
   */
  template <typename Scalar>
  Scalar as() const {
    try {
      return impl_.as<Scalar>();
    } catch (const YAML::Exception& exception) {
      throw Exception(std::string("Casting node '" + nestedKey_ + "' failed: ") + std::string(exception.what()));
    }
  }
};

//! Specialization of pushBack(..) for YAML nodes.
template <>
void YamlNode::pushBack(const YamlNode& t);

}  // namespace yaml_tools
