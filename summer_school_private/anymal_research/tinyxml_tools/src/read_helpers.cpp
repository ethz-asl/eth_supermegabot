/*!
 * @file	  read_helpers.cpp
 * @author	Gabriel Hottiger
 * @date	  Jan 10, 2017
 */

// tinyxml tools
#include <tinyxml_tools/read_helpers.hpp>

namespace tinyxml_tools {

TiXmlHandle getChildHandle(const TiXmlHandle& parentHandle, const std::string& childName, bool verbose) {
  TiXmlHandle childHandle(parentHandle);
  getChildHandle(childHandle, parentHandle, childName, verbose);
  return childHandle;
}

bool getChildHandle(TiXmlHandle& childHandle, TiXmlHandle parentHandle, std::string childName, bool verbose) {
  //! Recursively get parent handle
  size_t pos = 0;
  while ((pos = childName.find('/')) != std::string::npos) {
    if (!tinyxml_tools::getChildHandle(parentHandle, parentHandle, childName.substr(0, pos), verbose)) {
      if (verbose) {
        // Now check parent
        TiXmlElement* parentElement = parentHandle.ToElement();
        if (parentElement != nullptr) {
          MELO_WARN_STREAM("[getChildHandle] Could not get handle " << parentElement->ValueStr() << "::" << childName.substr(0, pos)
                                                                    << "! Returning parent handle!");
          childHandle = parentHandle;
        }
      }
      return false;
    }
    childName.erase(0, pos + 1);
  }

  // If handle exists return it
  if (parentHandle.FirstChild(childName).ToElement() == nullptr) {
    if (verbose) {
      MELO_WARN_STREAM("[getChildHandle] Could not get child handle for requested child " << childName << ".");
    }
    return false;
  }

  childHandle = parentHandle.FirstChild(childName);
  return true;
}

bool getChildElements(std::vector<TiXmlElement*>& childElements, const TiXmlHandle& parentHandle, const std::string& childName) {
  childElements.clear();

  // Check first child
  TiXmlHandle firstChildHandle = parentHandle;
  if (!getChildHandle(firstChildHandle, parentHandle, childName)) {
    return false;
  }
  auto childElement = firstChildHandle.Element();
  childElements.push_back(childElement);

  // Add siblings
  childElement = childElement->NextSiblingElement(childName);
  for (; childElement != nullptr; childElement = childElement->NextSiblingElement(childName)) {
    childElements.push_back(childElement);
  }

  return true;
}

TiXmlElement* getChildElement(TiXmlElement* parentElement, const std::string& childName) {
  return parentElement->FirstChildElement(childName);
}

}  // namespace tinyxml_tools
