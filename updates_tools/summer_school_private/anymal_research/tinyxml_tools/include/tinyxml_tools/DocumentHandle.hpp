/*!
 * @file	  DocumentHandle.hpp
 * @author	  Gabriel Hottiger
 * @date	  Jan 15, 2018
 */

#pragma once

// STL
#include <string>

namespace tinyxml_tools {

//! Modes in which document can be handled
enum class DocumentMode { NONE = -1, READ = 0, WRITE = 1, APPEND = 2 };
/**
 * @brief
 * @enum MERGE If files have equal attributes, they are NOT overwritten with the new values
 * @enum MERGE_OVERWRITE If files have equal attributes, they are overwritten with the new values
 * @enum REPLACE_OVERWRITE All attributes (of the file to be merged) need to be existing in the current file
 *                         and are then overwritten with the new values.
 */
enum class MergeMode { MERGE = 0, MERGE_OVERWRITE = 1, REPLACE_OVERWRITE = 2 };

class DocumentHandle {
 public:
  //! Default ctor
  DocumentHandle() : filepath_(""), mode_(DocumentMode::NONE) {}
  //! Default dtor
  virtual ~DocumentHandle() = default;

  /**
   * @brief Opens or creates a file at 'path' in depending on 'mode'
   * @param path Path of the config file to open/store
   * @param mode Access mode for the document
   * @return true, iff successful
   */
  virtual bool create(const std::string& path, DocumentMode mode) = 0;

  /**
   * @brief Merges file at 'path' with the handled document.
   * @param path Path of the config file to append
   * @param mode Mode of the merge, see Enum description on top of this file
   * @return true, iff successful
   */
  virtual bool merge(const std::string& path, MergeMode mode) = 0;

  /**
   * @brief Saves the current configuration to 'path'
   * @param path path of the output file
   * @param force force saving if file already exists
   * @return true, iff successful
   */
  virtual bool saveAs(const std::string& path, bool force) = 0;

  /**
   * @brief Saves the current configuration to the file specified in create()
   * @return true, iff successful
   */
  bool save() { return saveAs(filepath_, true); }

 protected:
  //! Check whether document can be saved (overwritten)
  bool writeable() { return mode_ == DocumentMode::APPEND || mode_ == DocumentMode::WRITE; }
  //! Check whether document shall be read on creation
  bool readable() { return mode_ == DocumentMode::APPEND || mode_ == DocumentMode::READ; }

 protected:
  //! Path of the handled file
  std::string filepath_;
  //! Document mode ( allows protecting files against reads/writes )
  DocumentMode mode_;
};

}  // namespace tinyxml_tools
