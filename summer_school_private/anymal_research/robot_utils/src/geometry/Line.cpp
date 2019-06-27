/*
 * Line.cpp
 *
 *  Created on: Apr 7, 2016
 *      Author: dbellicoso
 */

// robot utils
#include <robot_utils/geometry/Line.hpp>
#include <robot_utils/geometry/common/geometry_utils.hpp>
#include <robot_utils/math/math.hpp>

namespace robot_utils {
namespace geometry {

Line::Line() :
    Polygon()
{

}

Line::Line(const VertexList& vertices,
           robot_utils::geometry::VertexOrder vertexOrder) :
    Polygon(vertices, vertexOrder)
{
  assert(vertices_.size() == 2);
  lineCoefficients_ = Polygon::LineCoefficientList(1, {0.0, 0.0, 0.0});
}

Line::Line(VertexList&& vertices,
           robot_utils::geometry::VertexOrder vertexOrder) :
    Polygon(std::move(vertices), vertexOrder)
{
  assert(vertices_.size() == 2);
  lineCoefficients_ = Polygon::LineCoefficientList(1, {0.0, 0.0, 0.0});
}

double Line::getArea() const {
  return 0.0;
}

bool Line::isPointInPolygon(const Position2d& point, double tol) const {
  // Test inequalities implied by the line coefficients for all sides of the polygon.
  const double a = lineCoefficients_.front()[0];
  const double b = lineCoefficients_.front()[1];
  const double c = lineCoefficients_.front()[2];

  if (vertexOrder_ == robot_utils::geometry::VertexOrder::CounterClockWise) {
    // The point is on the line if
    //      ax + by = -c.
    return robot_utils::areNear(a*point.x() + b*point.y(), -c, tol);
  }
  else {
    // The point is on the line if
    //      ax + by = c.
    return robot_utils::areNear(a*point.x() + b*point.y(), c, tol);
  }
}


bool Line::isPointInLHSPlane(const Position2d& point) const {
  // Test inequalities implied by the line coefficients for all sides of the polygon.
  const double a = lineCoefficients_.front()[0];
  const double b = lineCoefficients_.front()[1];
  const double c = lineCoefficients_.front()[2];
  const double projection = a*point.x() + b*point.y();

  if (vertexOrder_ == robot_utils::geometry::VertexOrder::CounterClockWise) {
    // The point on the left hand side of the line if
    //      ax + by > -c.
    return (projection > -c);
  }
  else {
    // The point on the left hand side of the line if
    //      ax + by > c.
    return (projection > c);
  }
}


bool Line::updateLineCoefficients() {
  // Vertex order needs to be known in order to for the function to work
  if (vertexOrder_ == VertexOrder::Undefined) {
    return false;
  }
  return getLineCoefficientsFromVertices(vertices_[0], vertices_[1],
                                         lineCoefficients_[0], vertexOrder_);
}

double Line::getLineLength() const {
  return (vertices_[1] - vertices_[0]).norm();
}

bool Line::findIntersection(const Line& otherLine, Position2d& intersection) const {
  return findIntersection(otherLine, *this, intersection);
}

bool Line::findIntersection(const Line& lineA, const Line& lineB, Position2d& intersection) {
  // Check if line length is zero
  if ( lineA.getLineLength() == 0.0 || lineB.getLineLength() == 0.0 ) {
    return false;
  }

  const Eigen::Vector2d v1 = (lineA.getVertices()[1] - lineA.getVertices()[0]).normalized();
  const Eigen::Vector2d v2 = (lineB.getVertices()[1] - lineB.getVertices()[0]).normalized();

  // Check if v1 and v2 are parallel (matrix would not be invertible)
  const Position2d x = (Eigen::Matrix2d() << -v1, v2).finished().lu().solve(lineA.getVertices()[0] - lineB.getVertices()[1]);

  if (x(0)>=0.0 && x(0)<=1.0) {
    intersection << lineA.getVertices()[0].x() + x(0)*v1(0),
                    lineA.getVertices()[0].y() + x(0)*v1(1);
    return true;
  } else {
    return false;
  }
}

//bool Line::findIntersection(const Line& lineA, const Line& lineB, Position2d& intersection) {
//  // Check if line length is zero
//  if ( lineA.getLineLength() == 0.0 || lineB.getLineLength() == 0.0 ) {
//    std::cout << "Line::findIntersection: Line length is zero. Cannot compute intersection!\n";
//    return false;
//  }
//
//  // Line 1 is defined by (x1, y1), (x2, y2)
//  const double x1 = lineA.getVertices()[0].x();
//  const double y1 = lineA.getVertices()[0].y();
//  const double x2 = lineA.getVertices()[1].x();
//  const double y2 = lineA.getVertices()[1].y();
//
//  // Line 2 is defined by (x3, y3), (x4, y4)
//  const double x3 = lineB.getVertices()[0].x();
//  const double y3 = lineB.getVertices()[0].y();
//  const double x4 = lineB.getVertices()[1].x();
//  const double y4 = lineB.getVertices()[1].y();
//
//  // Determinant.
//  const double det = (x1-x2) * (y3-y4) - (y1-y2)*(x3-x4);
//
//  // Lines are parallel if determinante is zero
//  if (robot_utils::areNear(det, 0.0)) {
//    return false;
//  }
//
//  // Intersection
//  intersection.x() = (x1*y2-y1*x2) * (x3-x4) - (x1-x2) * (x3*y4-y3*x4);
//  intersection.x() /= det;
//
//  intersection.y() = (x1*y2-y1*x2) * (y3-y4) - (y1-y2) * (x3*y4-y3*x4);
//  intersection.y() /= det;
//
//  return true;
//}

bool Line::constructLargestConvexPolygonFromLines(const std::vector<Line>& lines, Polygon& convexHullPolygon) {
  if (lines.size()<2) {
    std::cout << "Polygon::constructConvexPolygonFromLines: Need at least two lines!\n";
    return false;
  }

  convexHullPolygon.resetVertices();
  bool success = true;
  unsigned int numOfLineIntersection = 0;

  for (unsigned int lineId=0; lineId<lines.size(); lineId++) {

    for (unsigned int lineIdNeighbour=0; lineIdNeighbour<lines.size(); lineIdNeighbour++) {
      // line cannot be it own neighbour
      if (lineId==lineIdNeighbour) {
        continue;
      }

      // Find intersection of lines
      Position2d intersection;
      if(!lines[lineId].findIntersection(lines[lineIdNeighbour], intersection)) {
        continue;
      }
      numOfLineIntersection++;
      convexHullPolygon.pushVertex(intersection);
    }
  }

  if (numOfLineIntersection<2) {
    std::cout << "Polygon::constructConvexPolygonFromLines: Found less than two line intersections!\n";
    return false;
  }

  return success;
}


bool Line::isPolygonOnRHSOfLine(const Polygon& polygon) const {

  for (const auto& vertex : polygon.getVertices()) {
    const double projection = lineCoefficients_.front()[0]*vertex.x() + lineCoefficients_.front()[1]*vertex.y();

    if (vertexOrder_ == robot_utils::geometry::VertexOrder::CounterClockWise) {
      // The point is on the LHS if ax + by >= -c.
      if ((projection >= -lineCoefficients_.front()[2])) {
        return false;
      }
    }
    else if (vertexOrder_ == robot_utils::geometry::VertexOrder::ClockWise) {
      // The point is on the LHS if ax + by >= c.
      if ((projection >= lineCoefficients_.front()[2])) {
        return false;
      }
    }
    else {
      std::cout << "[Polygon::isPolygonOnRHSOfLine] Undefined vertex order!\n";
      return false;
    }
  }

  return true;
}

bool Line::isPolygonVertexOnRHSOfLine(const Polygon& polygon) const {
  for (const auto& vertex : polygon.getVertices()) {
    const double a = lineCoefficients_.front()[0];
    const double b = lineCoefficients_.front()[1];
    const double c = lineCoefficients_.front()[2];
    const double projection = lineCoefficients_.front()[0]*vertex.x() + lineCoefficients_.front()[1]*vertex.y();

    if (vertexOrder_ == robot_utils::geometry::VertexOrder::CounterClockWise) {
      // The point is on the LHS if ax + by >= -c.
      if (!(projection >= -lineCoefficients_.front()[2])) {
        return true;
      }
    }
    else if (vertexOrder_ == robot_utils::geometry::VertexOrder::ClockWise) {
      // The point is on the LHS if ax + by >= c.
      if (!(projection >= lineCoefficients_.front()[2])) {
        return true;
      }
    }
    else {
      std::cout << "[Polygon::isPolygonVertexOnRHSOfLine] Undefined vertex order!\n";
      return false;
    }
  }

  return false;
}

bool Line::doesPolygonIntersectLine(const Polygon& polygon) const {
  return (!isPolygonOnRHSOfLine(polygon) && isPolygonVertexOnRHSOfLine(polygon));
}


} /* namespace robot_utils */
} /* namespace geometry */
