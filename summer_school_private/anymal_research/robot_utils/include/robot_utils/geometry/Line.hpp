/*
 * Line.hpp
 *
 *  Created on: Apr 7, 2016
 *      Author: dbellicoso
 */

#pragma once

// robot utils
#include <robot_utils/geometry/Polygon.hpp>

namespace robot_utils {
namespace geometry {

class Line : public Polygon
{
 public:
  Line();
  explicit Line(const VertexList& vertices,
       robot_utils::geometry::VertexOrder vertexOrder = VertexOrder::CounterClockWise);
  explicit Line(VertexList&& vertices,
       robot_utils::geometry::VertexOrder vertexOrder = VertexOrder::CounterClockWise);
  ~Line() override = default;

  double getArea() const override;
  bool updateLineCoefficients() override;
  bool isPointInPolygon(const Position2d& point, double tol=1e-6) const override;

  //! True if point is on the left hand side of the line.
  bool isPointInLHSPlane(const Position2d& point) const;

  double getLineLength() const;

  bool findIntersection(const Line& otherLine, Position2d& intersection) const;

  static bool findIntersection(const Line& lineA, const Line& lineB, Position2d& intersection);

  //! Given a set of lines, this function computes the vertices of the convex hull of all intersections
  bool constructLargestConvexPolygonFromLines(const std::vector<Line>& lines, Polygon& convexHullPolygon);

  //! True if the polygon lies completely on the RHS of the line.
  bool isPolygonOnRHSOfLine(const Polygon& polygon) const;

  //! True if at least one vertex of the polygon lies on the RHS of the line.
  bool isPolygonVertexOnRHSOfLine(const Polygon& polygon) const;

  //! True if line intersects the polygon.
  bool doesPolygonIntersectLine(const Polygon& polygon) const;
};

} /* namespace robot_utils */
} /* namespace geometry */
