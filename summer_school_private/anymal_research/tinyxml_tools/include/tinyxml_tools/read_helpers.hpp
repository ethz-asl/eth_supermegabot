/*!
 * @file	  read_helpers.hpp
 * @author	  Gabriel Hottiger
 * @date	  Jan 10, 2017
 */

#pragma once

// tinyxml_tools
#include "tinyxml_tools/traitsXML.hpp"

// message logger
#include "message_logger/message_logger.hpp"

// tiny xml
#include <tinyxml.h>

// Eigen
#include <Eigen/Core>

// STL
#include <string>
#include <type_traits>

namespace tinyxml_tools {

TiXmlHandle getChildHandle(const TiXmlHandle& parentHandle, const std::string& childName, bool verbose = true);

bool getChildHandle(TiXmlHandle& childHandle, TiXmlHandle parentHandle, std::string childName, bool verbose = true);

bool getChildElements(std::vector<TiXmlElement*>& childElements, const TiXmlHandle& parentHandle, const std::string& childName);

TiXmlElement* getChildElement(TiXmlElement* parentElement, const std::string& childName);

template <typename T, typename U = T, typename = typename std::enable_if<std::is_convertible<U, T>::value>::type>
bool loadParameter(T& parameter, const TiXmlElement* element,
                   const typename traits::element<T>::stringType& attributeName = traits::element<T>::getDefaultName(),
                   const U& defaultValue = traits::element<T>::getDefaultValue()) {
  // Check element
  if (!element) {
    MELO_WARN("[tinyxml_tools]: Element is null, set default value!");
    parameter = defaultValue;
    return false;
  }

  return traits::elementXML<T>::read(parameter, element, attributeName, defaultValue);
}

template <typename T, typename U = T,
          typename = typename std::enable_if<!std::is_const<T>::value && std::is_convertible<U, T>::value>::type>
bool loadParameter(T& parameter, const TiXmlHandle& handle,
                   const typename traits::element<T>::stringType& attributeName = traits::element<T>::getDefaultName(),
                   const U& defaultValue = traits::element<T>::getDefaultValue()) {
  return loadParameter(parameter, handle.Element(), attributeName, defaultValue);
}

template <typename T, typename U = T,
          typename = typename std::enable_if<!std::is_const<T>::value && std::is_convertible<U, T>::value>::type>
bool loadParameter(const std::string& name, T& parameter, const TiXmlHandle& parentHandle,
                   const typename traits::element<T>::stringType& attributeName = traits::element<T>::getDefaultName(),
                   const U& defaultValue = traits::element<T>::getDefaultValue()) {
  TiXmlHandle childHandle(parentHandle);
  if (!name.empty() && !getChildHandle(childHandle, parentHandle, name)) {
    return false;
  }
  return loadParameter(parameter, childHandle.Element(), attributeName, defaultValue);
}

template <typename T, typename = typename std::enable_if<!std::is_const<T>::value> >
bool loadParameterSiblings(T& parameter, TiXmlElement* element,
                           const typename traits::element<T>::siblingStringType& attributeName =
                               traits::element<typename traits::element<T>::siblingType>::getDefaultName(),
                           const typename traits::element<T>::siblingType& defaultValue =
                               traits::element<typename traits::element<T>::siblingType>::getDefaultValue()) {
  // Check element
  if (!element) {
    MELO_WARN("[tinyxml_tools]: Element is null, do nothing!");
    return false;
  }

  return traits::elementXML<T>::readSiblings(parameter, element, attributeName, defaultValue);
}

template <typename T, typename = typename std::enable_if<!std::is_const<T>::value> >
bool loadParameterSiblings(T& parameter, const TiXmlHandle& handle,
                           const typename traits::element<T>::siblingStringType& attributeName =
                               traits::element<typename traits::element<T>::siblingType>::getDefaultName(),
                           const typename traits::element<T>::siblingType& defaultValue =
                               traits::element<typename traits::element<T>::siblingType>::getDefaultValue()) {
  return loadParameterSiblings(parameter, handle.Element(), attributeName, defaultValue);
}

template <typename T, typename = typename std::enable_if<!std::is_const<T>::value> >
bool loadParameterSiblings(const std::string& name, T& parameter, const TiXmlHandle& parentHandle,
                           const typename traits::element<T>::siblingStringType& attributeName =
                               traits::element<typename traits::element<T>::siblingType>::getDefaultName(),
                           const typename traits::element<T>::siblingType& defaultValue =
                               traits::element<typename traits::element<T>::siblingType>::getDefaultValue()) {
  TiXmlHandle childHandle(parentHandle);
  if (!name.empty() && !getChildHandle(childHandle, parentHandle, name)) {
    return false;
  }
  return loadParameterSiblings(parameter, childHandle.Element(), attributeName, defaultValue);
}

}  // namespace tinyxml_tools
