/*
 * DistanceResults.hpp
 *
 *  Created on: Apr 4, 2018
 *      Author: Perry Franklin
 */

#pragma once

#include <vector>
#include <utility>
#include <map>

#include <collisions/CollisionBody.hpp>

#include <collisions_geometry/CollisionGeometry.hpp>

namespace collisions {

struct DistanceResult{

  using DistancePair = std::pair<unsigned int, unsigned int> ;

  DistancePair distance_pair;

  double distance;

  using Position = typename collisions_geometry::CollisionGeometry::Position;

  Position nearest_point_1;
  Position nearest_point_2;

};

// The DistanceResults class records results from a distance check. It tracks:
// Distances between all bodies.
// The locations of closest points.
class DistanceResults {
public:
  using DistancePair = typename DistanceResult::DistancePair;
  using Position = typename DistanceResult::Position;

 public:
  explicit DistanceResults( )= default;
  virtual ~DistanceResults() = default;

  inline std::map<DistancePair, DistanceResult>& getResults(){
    return distance_results_;
  }

  inline const std::map<DistancePair, DistanceResult>& getResults() const{
    return distance_results_;
  }

private:

  std::map<DistancePair, DistanceResult> distance_results_;

};

}
