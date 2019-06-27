/*!
 * @file	  write_helpers.cpp
 * @author	Gabriel Hottiger
 * @date	  Jan, 2018
 */

// tinyxml tools
#include <tinyxml_tools/write_helpers.hpp>

namespace tinyxml_tools {

bool createChildElement(TiXmlHandle& childHandle, const TiXmlHandle& parentHandle, std::string childName, bool forceIfExisting) {
  if (parentHandle.ToElement() == nullptr) {
    MELO_WARN_STREAM("[tinyxml_tools]: Could not create child handle '" << childName << "'. Parent element is null!");
    return false;
  }

  TiXmlHandle child = parentHandle;

  //! Recursively create child element
  size_t pos;
  do {
    pos = childName.find('/');
    std::string subName = childName.substr(0, pos);
    childName.erase(0, pos + 1);
    const bool exists = !(forceIfExisting && pos == std::string::npos) && child.FirstChild(subName).ToElement() != nullptr;
    child = exists ? child.FirstChild(subName).ToElement() : child.ToElement()->LinkEndChild(new TiXmlElement(subName))->ToElement();
  } while (pos != std::string::npos);

  childHandle = child;

  return true;
}

TiXmlHandle createChildElement(TiXmlHandle parentHandle, std::string childName, const bool forceIfExisting) {
  TiXmlHandle childHandle(parentHandle);
  createChildElement(childHandle, parentHandle, childName, forceIfExisting);
  return childHandle;
}

}  // namespace tinyxml_tools
