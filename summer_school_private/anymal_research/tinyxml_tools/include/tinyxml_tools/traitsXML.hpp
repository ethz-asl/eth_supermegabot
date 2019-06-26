/*!
 * @file	  traits.hpp
 * @author	Gabriel Hottiger
 * @date	  Jan, 2018
 */

#pragma once

// tinyxml_tools
#include <tinyxml_tools/traits.hpp>

// message logger
#include <message_logger/message_logger.hpp>

// tinyxml
#include <tinyxml.h>

// Eigen
#include <Eigen/Core>

// STL
#include <array>
#include <map>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace tinyxml_tools {

namespace traits {

//! Element trait for primitive types
template <typename T, typename Enable = void>
struct elementXML : public element<T> {
  // Forward types
  using Base = element<T>;
  using type = typename Base::type;
  using stringType = typename Base::stringType;

  // Read trait
  template <typename T_ = type>
  static bool read(type& value, const TiXmlElement* element, const stringType& name, const type& def,
                   typename std::enable_if<!std::is_same<T_, bool>::value>::type* = nullptr) {
    const bool success = element->QueryValueAttribute(name, &value) == TIXML_SUCCESS;
    if (!success) {
      MELO_WARN_STREAM("[tinyxml_tools]: Could not find attribute '" << name << "' in element: " << *element
                                                                     << ". Setting to default value '" << def << "'.");
    }
    return success;
  }

  template <typename T_ = type>
  static bool read(type& value, const TiXmlElement* element, const stringType& name, const type& def,
                   typename std::enable_if<std::is_same<T_, bool>::value>::type* = nullptr) {
    const bool success = element->QueryBoolAttribute(name.c_str(), &value) == TIXML_SUCCESS;
    if (!success) {
      MELO_WARN_STREAM("[tinyxml_tools]: Could not find attribute '" << name << "' in element: " << *element
                                                                     << ". Setting to default value '" << def << "'.");
    }
    return success;
  }

  // Write trait
  template <typename T_ = type>
  static void write(const type& value, TiXmlElement* const element, const stringType& name,
                    typename std::enable_if<!std::is_floating_point<T_>::value && !std::is_same<T_, char>::value>::type* = nullptr) {
    element->SetAttribute(name, value);
  }

  template <typename T_ = type>
  static void write(const type& value, TiXmlElement* const element, const stringType& name,
                    typename std::enable_if<std::is_floating_point<T_>::value>::type* = nullptr) {
    element->SetDoubleAttribute(name, value);
  }

  template <typename T_ = type>
  static void write(const type& value, TiXmlElement* const element, const stringType& name,
                    typename std::enable_if<std::is_same<T_, char>::value>::type* = nullptr) {
    element->SetAttribute(name, std::string(1, value));
  }
};

//! Element trait for stl pair
template <typename T, typename U>
struct elementXML<std::pair<T, U>> : public element<std::pair<T, U>> {
  // Forward types
  using Base = element<std::pair<T, U>>;
  using type = typename Base::type;
  using stringType = typename Base::stringType;

  // Read trait
  static bool read(type& value, const TiXmlElement* element, const stringType& name, const type& def) {
    bool success = true;
    success = traits::elementXML<T>::read(value.first, element, name.first, def.first) && success;
    success = traits::elementXML<U>::read(value.second, element, name.second, def.second) && success;
    return success;
  }

  // Write trait
  static void write(const type& value, TiXmlElement* const element, const stringType& name) {
    traits::elementXML<T>::write(value.first, element, name.first);
    traits::elementXML<U>::write(value.second, element, name.second);
  }
};

//! Element trait for stl vectors
template <typename T>
struct elementXML<std::vector<T>> : public element<std::vector<T>> {
  // Forward types
  using Base = element<std::vector<T>>;
  using type = typename Base::type;
  using stringType = typename Base::stringType;
  using siblingType = typename Base::siblingType;
  using siblingStringType = typename Base::siblingStringType;

  // Read trait
  static bool read(type& value, const TiXmlElement* element, const stringType& name, const type& def) {
    bool success = true;
    for (size_t i = 0; i < value.size(); ++i) {
      const std::string nameDef = name.empty() ? std::string("v_") + std::to_string(i) : name.at(i);
      const T defaultVal = def.empty() ? T{} : def.at(i);
      success = traits::elementXML<T>::read(value[i], element, nameDef, defaultVal) && success;
    }
    return success;
  }
  // Read siblings trait
  static bool readSiblings(type& value, TiXmlElement* element, const siblingStringType& name, const siblingType& def) {
    value.clear();
    bool success = true;
    TiXmlElement* siblingElement = element;
    do {
      siblingType val;
      success = traits::elementXML<siblingType>::read(val, siblingElement, name, def) && success;
      value.push_back(val);
      siblingElement = siblingElement->NextSiblingElement();
    } while (siblingElement != nullptr);
    return success;
  }

  // Write trait
  static void write(const type& value, TiXmlElement* const element, const stringType& name) {
    for (size_t i = 0; i < value.size(); ++i) {
      const std::string nameDef = name.empty() ? std::string("v_") + std::to_string(i) : name.at(i);
      traits::elementXML<T>::write(value[i], element, nameDef);
    }
  }
};

//! Element trait for stl arrays
template <typename T, std::size_t N>
struct elementXML<std::array<T, N>> : public element<std::array<T, N>> {
  // Forward types
  using Base = element<std::array<T, N>>;
  using type = typename Base::type;
  using stringType = typename Base::stringType;
  using siblingType = typename Base::siblingType;
  using siblingStringType = typename Base::siblingStringType;

  // Read trait
  static bool read(type& value, const TiXmlElement* element, const stringType& name, const type& def) {
    bool success = true;
    for (size_t i = 0; i < value.size(); ++i) {
      success = traits::elementXML<T>::read(value[i], element, name[i], def[i]) && success;
    }
    return success;
  }

  // Read siblings trait
  static bool readSiblings(type& value, TiXmlElement* element, const siblingStringType& name, const siblingType& def) {
    bool success = true;
    TiXmlElement* siblingElement = element;
    for (size_t i = 0; i < value.size(); ++i) {
      if (siblingElement == nullptr) {
        MELO_WARN_STREAM("[tinyxml_tools]: Not enough elements with attribute name '" << name << "' to load std::array of size " << N
                                                                                      << ".")
        return false;
      }
      success = traits::elementXML<siblingType>::read(value[i], siblingElement, name, def) && success;
      siblingElement = siblingElement->NextSiblingElement();
    }
    return success;
  }

  // Write trait
  static void write(const type& value, TiXmlElement* const element, const stringType& name) {
    for (size_t i = 0; i < value.size(); ++i) {
      traits::elementXML<T>::write(value[i], element, name[i]);
    }
  }
};

//! Element trait for stl maps
template <typename T>
struct elementXML<T, typename std::enable_if<map_helper::is_map<T>::value>::type> : public element<T> {
  // Forward types
  using Base = element<T>;
  using type = typename Base::type;
  using siblingType = typename Base::siblingType;
  using siblingStringType = typename Base::siblingStringType;

  // Read siblings trait
  static bool readSiblings(type& value, TiXmlElement* element, const siblingStringType& name, const siblingType& def) {
    value.clear();
    bool success = true;
    TiXmlElement* siblingElement = element;
    do {
      siblingType val;
      success = traits::elementXML<siblingType>::read(val, siblingElement, name, def) && success;
      value.insert(val);
      siblingElement = siblingElement->NextSiblingElement();
    } while (siblingElement != nullptr);
    return success;
  }
};

template <typename T>
struct elementXML<T, typename std::enable_if<std::is_base_of<typename Eigen::MatrixBase<T>, T>::value>::type> : public element<T> {
  // Forward types
  using Base = element<T>;
  using type = typename Base::type;
  using stringType = typename Base::stringType;

  // Read trait
  static bool read(type& value, const TiXmlElement* element, const stringType& name, const type& def) {
    bool success = true;
    for (int i = 0; i < value.rows(); ++i) {
      for (int j = 0; j < value.cols(); ++j) {
        const std::string nameDef =
            name.empty() ? (std::string("m_") + std::to_string(i) + std::to_string(j)) : name.at(i * value.cols() + j);
        success =
            traits::elementXML<typename T::Scalar>::read(value(i, j), element, nameDef, (def.size() ? def(i, j) : typename T::Scalar{})) &&
            success;
      }
    }
    return success;
  }

  // Write trait
  static void write(const type& value, TiXmlElement* const element, const stringType& name) {
    for (int i = 0; i < value.rows(); ++i) {
      for (int j = 0; j < value.cols(); ++j) {
        const std::string nameDef =
            name.empty() ? (std::string("m_") + std::to_string(i) + std::to_string(j)) : name.at(i * value.cols() + j);
        traits::elementXML<typename T::Scalar>::write(value(i, j), element, nameDef);
      }
    }
  }
};

}  // namespace traits

}  // namespace tinyxml_tools
