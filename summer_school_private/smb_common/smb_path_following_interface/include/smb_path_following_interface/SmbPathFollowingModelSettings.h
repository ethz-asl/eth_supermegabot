//
// Created by johannes on 29.04.19.
//

#pragma once

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <iostream>
#include <string>

namespace smb_path_following {

class SmbPathFollowingModelSettings {
 public:
  SmbPathFollowingModelSettings() : libraryFilesAreGenerated_(false) {}

  virtual ~SmbPathFollowingModelSettings() = default;

  /** flag to generate dynamic files **/
  bool libraryFilesAreGenerated_;

  virtual void loadSettings(const std::string& filename, bool verbose = true);
};

inline void SmbPathFollowingModelSettings::loadSettings(const std::string& filename, bool verbose /*= true*/) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(filename, pt);

  if (verbose) std::cerr << "\n #### Robot Model Settings:" << std::endl;
  if (verbose) std::cerr << " #### ==================================================" << std::endl;

  try {
    libraryFilesAreGenerated_ = pt.get<bool>("model_settings.libraryFilesAreGenerated");
    if (verbose) std::cerr << " #### libraryFilesAreGenerated ..... " << libraryFilesAreGenerated_ << std::endl;
  } catch (const std::exception& e) {
    if (verbose) std::cerr << " #### libraryFilesAreGenerated ..... " << libraryFilesAreGenerated_ << "\t(default)" << std::endl;
  }

  if (verbose) std::cerr << " #### ================================================ ####" << std::endl;
}

}  // namespace smb_path_following
