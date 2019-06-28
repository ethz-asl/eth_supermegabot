/*!
 * @file	  write_helpers.hpp
 * @author	Gabriel Hottiger
 * @date	  Jan, 2018
 */

#pragma once

// tinyxml_tools
#include "tinyxml_tools/traitsXML.hpp"

// message logger
#include "message_logger/message_logger.hpp"

// tiny xml
#include <tinyxml.h>

// STL
#include <string>

namespace tinyxml_tools {

bool createChildElement(TiXmlHandle& childHandle, const TiXmlHandle& parentHandle, std::string childName, bool forceIfExisting = false);

TiXmlHandle createChildElement(const TiXmlHandle& parentHandle, std::string childName, bool forceIfExisting = false);

template <typename T>
bool writeParameter(const T& parameter, TiXmlHandle handle,
                    const typename traits::element<T>::stringType& attributeName = traits::element<T>::getDefaultName()) {
  if (handle.ToElement() == nullptr) {
    return false;
  }
  traits::elementXML<T>::write(parameter, handle.ToElement(), attributeName);
  return true;
}

template <typename T>
bool writeParameter(const std::string& name, const T& parameter, TiXmlHandle parentHandle,
                    const typename traits::element<T>::stringType& attributeName = traits::element<T>::getDefaultName(),
                    const bool forceIfExisting = false) {
  TiXmlHandle childHandle(parentHandle);
  if (!name.empty() && !createChildElement(childHandle, parentHandle, name, forceIfExisting)) {
    return false;
  }
  return tinyxml_tools::writeParameter(parameter, childHandle, attributeName);
}

template <typename T>
bool writeParameterSiblings(const std::string& name, const T& parameter, TiXmlHandle parentHandle,
                            const typename traits::element<T>::siblingStringType& attributeName =
                                traits::element<typename traits::element<T>::siblingType>::getDefaultName()) {
  bool success = true;
  for (const auto& val : parameter) {
    success = writeParameter(name, val, parentHandle, attributeName, true) && success;
  }
  return success;
}

}  // namespace tinyxml_tools