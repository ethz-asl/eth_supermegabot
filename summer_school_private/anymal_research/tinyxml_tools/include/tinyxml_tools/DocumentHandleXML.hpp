/*!
 * @file	  DocumentHandleXML.hpp
 * @author	  Gabriel Hottiger
 * @date	  Jan 15, 2018
 */

#pragma once

// tinyxml_tools
#include <tinyxml_tools/DocumentHandle.hpp>
#include <tinyxml_tools/read_helpers.hpp>
#include <tinyxml_tools/traits.hpp>
#include <tinyxml_tools/traitsXML.hpp>
#include <tinyxml_tools/write_helpers.hpp>

// tiny_xml
#include <tinyxml.h>

// STL
#include <string>
#include <type_traits>

namespace tinyxml_tools {

class DocumentHandleXML : public DocumentHandle {
 public:
  //! Default ctor
  DocumentHandleXML();
  //! Default dtor
  ~DocumentHandleXML() override = default;
  TiXmlHandle getRootHandle() { return rootHandle_; }
  TiXmlHandle getDocumentHandle() { return &document_; }

  bool create(const std::string& path, DocumentMode mode) override { return create(path, mode, ""); }
  bool create(const std::string& path, DocumentMode mode, const std::string& rootName);

  bool merge(const std::string& path, MergeMode mode) override;

  bool saveAs(const std::string& path, bool force) override;

  bool seek(std::string handle);
  bool seekFromRoot(const std::string& handle);
  void reset();

  template <typename T, typename U = T,
            typename = typename std::enable_if<!std::is_const<T>::value && std::is_convertible<U, T>::value>::type>
  bool read(const std::string& name, T& value,
            const typename traits::element<T>::stringType& attributeName = traits::element<T>::getDefaultName(),
            const U& defaultValue = traits::element<T>::getDefaultValue()) const {
    return loadParameter<T>(name, value, seekedHandle_, attributeName, defaultValue);
  }

  template <typename T, typename U = T,
            typename = typename std::enable_if<!std::is_const<T>::value && std::is_convertible<U, T>::value>::type>
  bool readAttributes(T& value, const typename traits::element<T>::stringType& attributeName = traits::element<T>::getDefaultName(),
                      const U& defaultValue = traits::element<T>::getDefaultValue()) const {
    return read<T>("", value, attributeName, defaultValue);
  }

  template <typename T, typename = typename std::enable_if<!std::is_const<T>::value> >
  bool readSiblings(const std::string& name, T& value,
                    const typename traits::element<T>::siblingStringType& attributeName =
                        traits::element<typename traits::element<T>::siblingType>::getDefaultName(),
                    const typename traits::element<T>::siblingType& defaultValue =
                        traits::element<typename traits::element<T>::siblingType>::getDefaultValue()) const {
    return loadParameterSiblings<T>(name, value, seekedHandle_, attributeName, defaultValue);
  }

  template <typename T, typename = typename std::enable_if<!std::is_const<T>::value> >
  bool readSiblingAttributes(T& value,
                             const typename traits::element<T>::siblingStringType& attributeName =
                                 traits::element<typename traits::element<T>::siblingType>::getDefaultName(),
                             const typename traits::element<T>::siblingType& defaultValue =
                                 traits::element<typename traits::element<T>::siblingType>::getDefaultValue()) const {
    return readSiblings<T>("", value, attributeName, defaultValue);
  }

  template <typename T>
  bool write(const std::string& name, const T& value,
             const typename traits::element<T>::stringType& attributeName = traits::element<T>::getDefaultName()) {
    return writeParameter<T>(name, value, seekedHandle_, attributeName);
  }

  template <typename T>
  bool writeAttributes(const T& value,
                       const typename traits::element<T>::stringType& attributeName = traits::element<T>::getDefaultName()) {
    return write<T>("", value, attributeName);
  }

  template <typename T>
  bool writeSiblings(const std::string& name, const T& value,
                     const typename traits::element<T>::siblingStringType& attributeName =
                         traits::element<typename traits::element<T>::siblingType>::getDefaultName()) {
    return writeParameterSiblings<T>(name, value, seekedHandle_, attributeName);
  }

 protected:
  bool resolveDefault();
  bool resolveIncludes();
  bool resolveExtensions();
  virtual std::string convertPath(const std::string& path);
  bool merge(const std::string& path, MergeMode mode, bool force);
  bool mergeTrees(TiXmlHandle tree1, TiXmlHandle tree2, MergeMode mode);
  void findElements(std::vector<TiXmlHandle>& handles, TiXmlHandle tree, std::string elementName);
  const TiXmlDocument& getDocument() { return document_; }

 protected:
  TiXmlDocument document_;
  TiXmlHandle rootHandle_;
  TiXmlHandle seekedHandle_;
};  // namespace tinyxml_tools

}  // namespace tinyxml_tools
