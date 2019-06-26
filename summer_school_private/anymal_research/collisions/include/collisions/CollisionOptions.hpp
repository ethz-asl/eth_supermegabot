/*
 * ColliderOptions.hpp
 *
 *  Created on: May 15, 2017
 *      Author: Perry Franklin
 */

#pragma once

namespace collisions {

class CollisionOptions {

 public:
  explicit CollisionOptions():
    dont_calculate_contacts(false),
    ignore_colgeom_margin(false),
    extra_margin(0.0) {};
  virtual ~CollisionOptions() = default;

  bool dont_calculate_contacts;

  bool ignore_colgeom_margin;

  double extra_margin;

};

}
