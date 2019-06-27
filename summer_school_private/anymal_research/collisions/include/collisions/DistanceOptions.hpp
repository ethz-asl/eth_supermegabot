/*
 * DistanceOptions.hpp
 *
 *  Created on: Apr 20, 2018
 *      Author: Perry Franklin
 */

#pragma once


namespace collisions {

class DistanceOptions {

 public:
  explicit DistanceOptions():
    only_smallest_distance(false),
    ignore_colgeom_margin(false),
    extra_margin(0.0) {};
  virtual ~DistanceOptions() = default;

  bool only_smallest_distance;

  bool ignore_colgeom_margin;

  double extra_margin;

};

}




