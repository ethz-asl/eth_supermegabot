/*
 * Surface.hpp
 *
 *  Created on: Mar 21, 2015
 *      Author: C. Dario Bellicoso
 */

#pragma once

// kindr
#include "kindr/Core"

namespace robot_utils {

class Surface
{
 public:
  typedef kindr::VectorTypeless3D Vector;
  Surface();
  virtual ~Surface();
};

} /* namespace robot_utils */
