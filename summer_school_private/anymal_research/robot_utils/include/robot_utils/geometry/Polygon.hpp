/*
 * Polygon.hpp
 *
 *  Created on: Apr 7, 2016
 *      Author: Dario Bellicoso
 */


#pragma once

// eigen
#include <Eigen/Core>
#include <Eigen/StdVector>

// stl
#include <vector>
#include <list>

// kindr
#include <kindr/Core>

// robot utils
#include "robot_utils/geometry/common/geometry_utils.hpp"

namespace robot_utils {
namespace geometry {

class Polygon {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using VertexList = std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>;
  using LineCoefficientList = std::vector<std::vector<double>>;

  Polygon();

  explicit Polygon(const VertexList& vertices,
          robot_utils::geometry::VertexOrder vertexOrder = robot_utils::geometry::VertexOrder::Undefined);

  explicit Polygon(VertexList&& vertices,
          robot_utils::geometry::VertexOrder vertexOrder = robot_utils::geometry::VertexOrder::Undefined);

  Polygon(const Position2d& vertexCenter,
          unsigned int numOfVertices,
          double polygonSize);

  Polygon(const Position2d& vertexCenter,
          unsigned int numOfVertices,
          double polygonSize,
          const Eigen::Matrix2d& RotationMatrix2d);

  Polygon(const Polygon& polygon);

  Polygon(Polygon&& rhs) = default;

  Polygon& operator=(const Polygon& rhs);

  Polygon& operator=(Polygon&& rhs) = default;

  virtual ~Polygon() = default;

  //! Get the area of the polygon (is negative if order is clock wise).
  virtual double getArea() const;

  //! Get the perimeter of the polygon.
  double getPerimeter() const;

  //! Get the number of vertices.
  size_t getNumVertices() const;

  //! Copy a vertex to the vertex list at a specific index.
  void setVertex(const Position2d& vertex, unsigned int vertexId);

  //! Copy a vertex to the vertex list at a specific index.
  const Position2d& getVertex(unsigned int vertexId) const;

  //! Add a vertex to the vertex list.
  void pushVertex(const Position2d& vertex);

  //! Reorder the vertices in clockwise or counter-clockwise order.
  bool reorderVertices(VertexOrder vertexOrder);

  //! Get the order in which the vertices are stored.
  VertexOrder getVertexOrder() const;

  //! Compute the line coefficients of all edges of the polygon.
  virtual bool updateLineCoefficients();

  //! Get the set of line coefficients of all edges of the polygon.
  const LineCoefficientList& getLineCoefficients() const;

  //! Get the set of vertices.
  const VertexList& getVertices() const;

  //! Check if a point is in the polygon.
  virtual bool isPointInPolygon(const Position2d& point, double tol=1e-6) const;

  //! Find whether two polygons intersect. This method assumes that both polygons have been reordered either clockwise or counterclockwise.
  //! This is an implementation of the SAT algorithm
  // http://www.dyn4j.org/2010/01/sat/.
  static bool doPolygonsIntersect(const Polygon& polygonA, const Polygon& polygonB);

  //! Project the vertices of a polygon to a scalar range directed along an axis.
  static void projectPolygonToRange(
      const Polygon& polygon,
      const Position2d& axisDirection,
      Eigen::Vector2d& range);

  //! Check if two scalar ranges A = {a, b} and B = {c, d} overlap.
  static bool doRangesOverlap(
      const Position2d& rangeA,
      const Position2d& rangeB);

  //! The edges of the polygon are moved with towards the incenter with a certain distance.
  bool resizePolygonTowardsInCenter(double lengthLineToLine);

  //! The polygon center is defined to be the centroid (average of all vertices). Vertices are
  // shrunk towards this center with a certain amount of percentage.
  bool resizeVerticesTowardsCentroid(double percent);

  //! For each polygon, we extract one (random) triangle and move its vertices towards the incenter
  // for some percentage. The resulting distance is used for the other vertices aswell.
  bool resizeAreaTriangleBased(double percent);

  //! Emtpy the vertex list.
  void resetVertices(unsigned int numOfVertices = 0u);

  //! Print information about the polygon.
  void print() const;

  //! Compute centroid (polygon center).
  bool computePolygonCentroid(Eigen::Vector2d& center) const;

  //! Compute point on a polygon side that is closest to a point.
  // Note: The function assumes that point is located outside of the polygon and
  // that the polygon is convex. For efficiency reasons, no checks have been added.
  bool computeClosestPointOnPolygon(Position2d& pointOPolygon, const Position2d& point) const;


 protected:
  //! A list of vertices.
  VertexList vertices_;

  //! The order in which the vertices are stored.
  robot_utils::geometry::VertexOrder vertexOrder_;

  //! The set of coefficients of the edges of the polygon.
  LineCoefficientList lineCoefficients_;

};

} /* namespace robot_utils */
} /* namespace geometry */
