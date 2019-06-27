/*!
 * @file	  DocumentHandleXML.cpp
 * @author	  Gabriel Hottiger
 * @date	  Jan 15, 2018
 */

#include "tinyxml_tools/DocumentHandleXML.hpp"

#include <sys/stat.h>

namespace tinyxml_tools {

DocumentHandleXML::DocumentHandleXML() : DocumentHandle(), document_(), rootHandle_(&document_), seekedHandle_(&document_) {}

bool DocumentHandleXML::create(const std::string& path, DocumentMode mode, const std::string& rootName) {
  // Set members
  mode_ = mode;
  filepath_ = path;

  bool success = false;

  // Read file
  if (readable()) {
    // Open file (LoadFile does clear internally)
    success = document_.LoadFile(filepath_) && document_.RootElement() != nullptr;
    if (success && document_.RootElement()->NextSiblingElement() != nullptr) {
      MELO_ERROR_STREAM("[tinyxml_tools::DocumentHandle] Could not create file " << path << ", has multiple roots.");
      return false;
    }
  }

  // If append and read was not successful write new file
  if (writeable() && !success) {
    // Clear document and add declaration
    document_.Clear();
    document_.LinkEndChild(new TiXmlDeclaration("1.0", "", ""));
    if (!rootName.empty()) {
      document_.LinkEndChild(new TiXmlElement(rootName));
    }
    success = true;
  } else if (!writeable() && !success) {
    MELO_ERROR_STREAM(
        "[tinyxml_tools::DocumentHandle] create(...) was not able to read the file, and cannot create a new file as a non-writeable mode "
        "was chosen");
    return false;
  }

  if (mode == DocumentMode::NONE) {
    document_.Clear();
    MELO_ERROR_STREAM("[tinyxml_tools::DocumentHandle] Could not create file " << path << ", mode is NONE.");
    return false;
  }

  // Reset handles
  reset();

  // Resolve includes
  success = success && resolveIncludes();
  success = success && resolveDefault();
  success = success && resolveExtensions();

  return success;
}

bool DocumentHandleXML::merge(const std::string& path, MergeMode mode, bool force) {
  // Load file to merge in
  DocumentHandleXML mergeDocumentHandle;
  if (!mergeDocumentHandle.create(path, DocumentMode::READ)) {
    MELO_ERROR_STREAM("[tinyxml_tools::DocumentHandle] merge(...) could not read the file at " << path);
    return false;
  }
  auto mergeDocument = mergeDocumentHandle.getDocument();

  // Check if current document allows writing
  if (!force && !writeable()) {
    MELO_ERROR_STREAM("[tinyxml_tools::DocumentHandle] Could not merge file " << path << ". Document is read-only.");
    return false;
  }

  // Check if current file is empty
  if (document_.RootElement() == nullptr) {
    document_.LinkEndChild(&mergeDocument);
    reset();
    return true;
  }

  // Check identical roots
  if (document_.RootElement()->ValueStr() != mergeDocument.RootElement()->ValueStr()) {
    MELO_ERROR_STREAM("[tinyxml_tools::DocumentHandle] Could not merge file " << path << ". Documents have different Roots.");
    return false;
  }

  // Call recursion on temporary object, save it to document on success
  bool success = mergeTrees(document_.RootElement(), mergeDocument.RootElement(), mode);
  if (success) {
    reset();
    success &= resolveIncludes();
    success &= resolveDefault();
    success &= resolveExtensions();
  }

  return success;
}

bool DocumentHandleXML::merge(const std::string& path, MergeMode mode) { return merge(path, mode, false); }

bool DocumentHandleXML::saveAs(const std::string& path, bool force) {
  // Check validity of request
  if (!writeable()) {
    MELO_WARN_STREAM("[tinyxml_tools::DocumentHandle] Can not save file " << path << ". Read-only.");
    return false;
  }

  // Check if it is current file and writeable
  if ((path == filepath_) && force && !writeable()) {
    MELO_WARN_STREAM("[tinyxml_tools::DocumentHandle] Can not save file " << path << ".");
    return false;
  }
  // Check if file exists
  struct stat buffer {};
  bool fileExists = (stat(path.c_str(), &buffer) == 0);
  if (fileExists && !force) {
    MELO_WARN_STREAM("[tinyxml_tools::DocumentHandle] Could not save file " << path << " file exists.");
    return false;
  }
  return document_.SaveFile(path);
}

bool DocumentHandleXML::seek(std::string handle) {
  if (seekedHandle_.ToElement() == nullptr) {
    MELO_WARN_STREAM("[tinyxml_tools::DocumentHandle] Corrupted seeked handle. Reset it to root handle.");
    seekedHandle_ = rootHandle_;
    return false;
  }

  while (handle.substr(0, 3) == std::string("../")) {
    if (seekedHandle_.ToElement()->Parent() == nullptr) {
      MELO_WARN_STREAM("[tinyxml_tools::DocumentHandle] No parent handle when seeking " << handle
                                                                                        << ". Reset seeked handle to root handle.");
      seekedHandle_ = rootHandle_;
      return false;
    }
    // Set parent
    handle.erase(0, 3);
    seekedHandle_ = seekedHandle_.ToElement()->Parent();
  }

  if (!handle.empty()) {
    if (!getChildHandle(seekedHandle_, seekedHandle_, handle)) {
      MELO_WARN_STREAM("[tinyxml_tools::DocumentHandle] Seeked handle " << handle
                                                                        << " does not exist. Reset seeked handle to root handle.");
      seekedHandle_ = rootHandle_;
      return false;
    }
  }

  return true;
}

bool DocumentHandleXML::seekFromRoot(const std::string& handle) {
  if (handle.empty()) {
    seekedHandle_ = rootHandle_;
    return true;
  }
  return getChildHandle(seekedHandle_, rootHandle_, handle);
}

void DocumentHandleXML::reset() {
  rootHandle_ = document_.RootElement();
  seekedHandle_ = rootHandle_;
}

std::string DocumentHandleXML::convertPath(const std::string& path) {
  if (path.at(0) == '/' || path.substr(0, 2) == "./") {
    return path;
  } else {
    std::string tmp;
    auto pos = filepath_.rfind("/");
    if (pos != std::string::npos) {
      tmp = filepath_.substr(0, pos + 1);
    }
    tmp += path;
    return tmp;
  }
}

bool DocumentHandleXML::resolveDefault() {
  // No root element ill-formed xml
  if (rootHandle_.ToElement() == nullptr) {
    return true;
  }

  const char* attributeString = rootHandle_.ToElement()->Attribute("default");
  if (attributeString == nullptr) {
    return true;
  }
  std::string defaultPath = convertPath(attributeString);
  rootHandle_.ToElement()->RemoveAttribute("default");

  // Load file to merge in
  DocumentHandleXML defaultDocumentHandle;
  if (!defaultDocumentHandle.create(defaultPath, DocumentMode::READ)) {
    MELO_ERROR_STREAM("[tinyxml_tools::DocumentHandle] Could not load default file " << defaultPath << ". Could not load file.");
    return false;
  }

  // Replace merge
  if (!mergeTrees(defaultDocumentHandle.rootHandle_, rootHandle_, MergeMode::REPLACE_OVERWRITE)) {
    MELO_ERROR_STREAM("[tinyxml_tools::DocumentHandle] resolveDefault() failed to merge the trees.");
    return false;
  }
  rootHandle_.ToElement()->Clear();
  for (auto* child = defaultDocumentHandle.rootHandle_.FirstChild().ToElement(); child != nullptr; child = child->NextSiblingElement()) {
    rootHandle_.ToElement()->LinkEndChild(new TiXmlElement(*child));
  }
  return true;
}

bool DocumentHandleXML::resolveExtensions() {
  // No root element ill-formed xml
  if (rootHandle_.ToElement() == nullptr) {
    return true;
  }

  const char* attributeString = rootHandle_.ToElement()->Attribute("extension");
  if (attributeString == nullptr) {
    return true;
  }
  std::string extensionPath = convertPath(attributeString);
  rootHandle_.ToElement()->RemoveAttribute("extension");

  return merge(extensionPath, MergeMode::MERGE_OVERWRITE, true);
}

bool DocumentHandleXML::resolveIncludes() {
  // Find all include_xml tags
  std::vector<TiXmlHandle> handles;
  findElements(handles, rootHandle_, "include_xml");

  for (auto& handle : handles) {
    if (handle.ToElement() != nullptr && handle.ToElement()->Parent()->ToElement() != nullptr) {
      const char* attributeString = handle.ToElement()->Attribute("dir");
      if (attributeString == nullptr) {
        MELO_WARN_STREAM("[tinyxml_tools::DocumentHandle]: Could not include file, no attribute dir in tag include_xml.");
        continue;
      }

      // Load file to merge in
      std::string includePath = convertPath(attributeString);
      DocumentHandleXML includeDocumentHandle;
      if (!includeDocumentHandle.create(includePath, DocumentMode::READ)) {
        MELO_ERROR_STREAM("[tinyxml_tools::DocumentHandle] Could not include file " << includePath << ". Could not load file.");
        return false;
      }
      auto includeDocument = includeDocumentHandle.getDocument();

      // Remove include tag
      TiXmlHandle parent = handle.ToElement()->Parent();
      parent.ToElement()->RemoveChild(handle.ToElement());

      // Check if root is name "ignore_root"
      auto includeRoot = includeDocument.RootElement();
      if (includeRoot->ValueStr() == std::string{"ignore_root"}) {
        // Merge all childs
        for (auto childElement = includeRoot->FirstChildElement(); childElement != nullptr;
             childElement = childElement->NextSiblingElement()) {
          parent.ToElement()->LinkEndChild(new TiXmlElement(*childElement));
        }
      } else {
        // Merge root
        parent.ToElement()->LinkEndChild(new TiXmlElement(*includeRoot));
      }

    } else {
      MELO_WARN_STREAM("[tinyxml_tools::DocumentHandle]: Could not include file, corrupted handle.");
    }
  }

  return true;
}

void DocumentHandleXML::findElements(std::vector<TiXmlHandle>& handles, TiXmlHandle tree, const std::string elementName) {
  // Recurse
  for (TiXmlElement* childElement = tree.ToElement(); childElement != nullptr; childElement = childElement->NextSiblingElement()) {
    if (std::string(childElement->Value()) == elementName) {
      handles.emplace_back(childElement);
    }
    findElements(handles, childElement->FirstChild(), elementName);
  }
}

bool DocumentHandleXML::mergeTrees(TiXmlHandle tree1, TiXmlHandle tree2, MergeMode mode) {
  if (tree1.ToElement() == nullptr) {
    return false;
  }

  // Loop over tree2 childs
  for (TiXmlElement* tree2ChildElement = tree2.FirstChild().ToElement(); tree2ChildElement != nullptr;
       tree2ChildElement = tree2ChildElement->NextSiblingElement()) {
    // Check structure
    TiXmlElement* tree1ChildElement = tree1.FirstChild(tree2ChildElement->Value()).ToElement();
    if (tree1ChildElement == nullptr) {
      if (mode == MergeMode::REPLACE_OVERWRITE) {
        MELO_ERROR_STREAM("[tinyxml_tools::DocumentHandle] Structure mismatch element " << tree2ChildElement->Value()
                                                                                        << " does not exist!");
        return false;
      } else {
        tree1.ToElement()->LinkEndChild(new TiXmlElement(*tree2ChildElement));
        return true;
      }
    } else {
      // Set attributes
      for (TiXmlAttribute* attribute = tree2ChildElement->FirstAttribute(); attribute != nullptr; attribute = attribute->Next()) {
        // Check attribute
        if (tree1ChildElement->Attribute(attribute->Name()) == nullptr) {
          if (mode == MergeMode::REPLACE_OVERWRITE) {
            MELO_ERROR_STREAM("[tinyxml_tools::DocumentHandle] Structure mismatch attribute " << attribute->Name() << " does not exist!");
            return false;
          } else {
            tree1ChildElement->SetAttribute(attribute->Name(), attribute->ValueStr());
          }
        } else {
          if (mode == MergeMode::REPLACE_OVERWRITE || mode == MergeMode::MERGE_OVERWRITE) {
            tree1ChildElement->SetAttribute(attribute->Name(), attribute->ValueStr());
          }
        }
      }
    }

    if (!mergeTrees(tree1ChildElement, tree2ChildElement, mode)) {
      return false;
    }
  }
  return true;
}

}  // namespace tinyxml_tools
