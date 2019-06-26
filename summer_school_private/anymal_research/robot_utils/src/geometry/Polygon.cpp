/*
 * Polygon.cpp
 *
 *  Created on: Apr 7, 2016
 *      Author: Dario Bellicoso
 */

// robot utils
#include "robot_utils/geometry/Polygon.hpp"
#include "robot_utils/geometry/Line.hpp"
#include <robot_utils/math/math.hpp>

#include <boost/assert.hpp>

namespace robot_utils {
namespace geometry {

Polygon::Polygon()
    : vertices_(),
      vertexOrder_(VertexOrder::Undefined),
      lineCoefficients_()
{

}


Polygon::Polygon(const VertexList& vertices,
                 robot_utils::geometry::VertexOrder vertexOrder)
  : vertices_(vertices.begin(), vertices.end()),
    vertexOrder_(vertexOrder),
    lineCoefficients_()
{

}

Polygon::Polygon(VertexList&& vertices,
                 robot_utils::geometry::VertexOrder vertexOrder)
  : vertexOrder_(vertexOrder),
    lineCoefficients_()
{
  vertices_ = std::move(vertices);
}

Polygon::Polygon(const Position2d& vertexCenter,
                 unsigned int numOfVertices,
                 double polygonSize) :
    vertexOrder_(VertexOrder::CounterClockWise),
    lineCoefficients_()
{
  assert(numOfVertices >= 2u);
  vertices_.resize(numOfVertices);

  // angle between two vertices
  const double yawAngle = 2.0*M_PI/( static_cast<double>(numOfVertices));

  // set up rotation matrix (it rotates one vertex into another)
  Eigen::Matrix2d RotationMatrix2d;
  RotationMatrix2d <<
      cos(yawAngle), -sin(yawAngle),
      sin(yawAngle),  cos(yawAngle);

  // Construct polygon.
  Position2d vertexOffset(polygonSize, 0.0);
  for (auto& vertex : vertices_) {
    vertex = vertexCenter + vertexOffset;

    // rotate vertex vector (Note: copy for avoiding aliasing)
    const Position2d previousVertexOffset = vertexOffset;
    vertexOffset = RotationMatrix2d*previousVertexOffset;
  }
}

Polygon::Polygon(const Position2d& vertexCenter,
                 unsigned int numOfVertices,
                 double polygonSize,
                 const Eigen::Matrix2d& RotationMatrix2d) :
    vertexOrder_(VertexOrder::CounterClockWise),
    lineCoefficients_()
{
  assert(numOfVertices >= 2u);
  vertices_.resize(numOfVertices);

  // Construct polygon.
  Position2d vertexOffset(polygonSize, 0.0);
  for (auto& vertex : vertices_) {
    vertex = vertexCenter + vertexOffset;

    // rotate vertex vector (Note: copy for avoiding aliasing)
    const Position2d previousVertexOffset = vertexOffset;
    vertexOffset = RotationMatrix2d*previousVertexOffset;
  }
}

Polygon::Polygon(const Polygon& polygon)
    : vertices_(polygon.getVertices()),
      vertexOrder_(polygon.getVertexOrder()),
      lineCoefficients_(polygon.getLineCoefficients()) {

}

Polygon& Polygon::operator=(const Polygon& rhs) {
  if(this == &rhs) {
    return *this;
  }

  this->vertices_         = rhs.getVertices();
  this->vertexOrder_      = rhs.getVertexOrder();
  this->lineCoefficients_ = rhs.getLineCoefficients();
  return *this;
}

bool Polygon::reorderVertices(VertexOrder vertexOrder) {
  if (vertices_.size()<=2u) {
    vertexOrder_ = vertexOrder;
    return true;
  }

  bool success = true;

  // Compute polygon center.
  Position2d polygonCenter;
  success &= computePolygonCentroid(polygonCenter);

  // Sort in clockwise/counter clockwise order.
  std::sort(vertices_.begin(), vertices_.end(),
    [&polygonCenter, &vertexOrder](const Position2d& p1,
                                   const Position2d& p2) -> bool {
      const bool isLess = less(p1, p2, polygonCenter);
      if (vertexOrder == VertexOrder::ClockWise) {
        return !isLess;
      } else if (vertexOrder == VertexOrder::CounterClockWise) {
        return isLess;
      }
  });

  // Allocate.
  vertexOrder_ = vertexOrder;
  return success;
}

VertexOrder Polygon::getVertexOrder() const {
  return vertexOrder_;
}

size_t Polygon::getNumVertices() const {
  return vertices_.size();
}

double Polygon::getArea() const {
  if (vertices_.size()<=2u) {
    return 0.0;
  }

  if (vertexOrder_ == VertexOrder::Undefined) {
    std::cout << "[Polygon::getArea] Undefined vertex order\n";
    return 0.0;
  }

  // Compute area using shoelace formula.
  double area = 0.0;
  for (unsigned int k=0u; k<vertices_.size(); ++k) {
    const auto kNext = robot_utils::intmod(k+1, vertices_.size());
    area += det(vertices_[k], vertices_[kNext]);
  }
  return 0.5*area;
}

double Polygon::getPerimeter() const {
  if (vertices_.size() <= 1u) {
    return 0.0;
  }

  else if (vertices_.size() == 2u) {
    return (vertices_[0u] - vertices_[1u]).norm();
  }

  if (vertexOrder_ == VertexOrder::Undefined) {
    std::cout << "[Polygon::getPerimeter] Undefined vertex order\n";
    return 0.0;
  }

  double perimeter = 0.0;
  for (unsigned int k=0u; k<vertices_.size(); ++k) {
    perimeter += (
          vertices_[robot_utils::intmod(k+1, vertices_.size())] -
          vertices_[k]
        ).norm();
  }

  return perimeter;
}

void Polygon::setVertex(const Position2d& position, unsigned int vertexId) {
  assert(vertexId>=0u && vertexId<vertices_.size());
  vertices_[vertexId] = position;
}

const Position2d& Polygon::getVertex(unsigned int vertexId) const {
  assert(vertexId>=0u && vertexId<vertices_.size());
  return vertices_[vertexId];
}

void Polygon::pushVertex(const Position2d& position) {
  vertices_.emplace_back(position);
}

const Polygon::LineCoefficientList& Polygon::getLineCoefficients() const {
  return lineCoefficients_;
}

const Polygon::VertexList& Polygon::getVertices() const {
  return vertices_;
}

bool Polygon::isPointInPolygon(const Position2d& point, double tol) const {
  if (vertices_.size() == 0u) {
    return false;
  } else if (vertices_.size() == 1u) {
    return ( (point-vertices_.front()).norm() == 0.0 );
  }

  // Test inequalities implied by the line coefficients for all sides of the polygon.
  for (unsigned int lineId = 0u; lineId<lineCoefficients_.size(); ++lineId) {
    const double projection = lineCoefficients_[lineId][0]*point.x() + lineCoefficients_[lineId][1]*point.y();

    if (vertexOrder_ == robot_utils::geometry::VertexOrder::CounterClockWise) {
      // The point is in or on the polygon if for all lines it holds that
      //      ax + by >= -c.
      if (!(projection >= -lineCoefficients_[lineId][2]-tol)) {
      	return false;
      }
    }
    else if (vertexOrder_ == robot_utils::geometry::VertexOrder::ClockWise) {
      // The point is in or on the polygon if for all lines it holds that:
      //      ax + by >= c.
      if (!(projection >= lineCoefficients_[lineId][2]-tol)) {
        return false;
      }
    }
    else {
      std::cout << "[Polygon::isPointInPolygon] Undefined vertex order!\n";
      return false;
    }
  }

  return true;
}


bool Polygon::updateLineCoefficients() {
  if (vertices_.size() <= 1u) {
    return true;
  }

  else if (vertices_.size() == 2u ) {
    lineCoefficients_ = LineCoefficientList(1u, std::vector<double>(3, 0.0));
    return getLineCoefficientsFromVertices(vertices_[0], vertices_[1],
                                          lineCoefficients_[0], vertexOrder_);
  }

  bool success = true;
  lineCoefficients_ = LineCoefficientList(vertices_.size(), std::vector<double>(3, 0.0));

  if (vertexOrder_ == VertexOrder::Undefined) {
    std::cout << "[Polygon::isPointInPolygon] Undefined vertex order!\n";
    return false;
  }

  for (unsigned int vId = 0u; vId < vertices_.size(); ++vId) {
    const unsigned int vIdNext = robot_utils::intmod(vId+1, vertices_.size());
    BOOST_ASSERT_MSG(vIdNext < vertices_.size(), ("vIdNext was: " + std::to_string(vIdNext) + " numVertices was: " + std::to_string(vertices_.size())).c_str());
    BOOST_ASSERT_MSG(vIdNext >= 0, ("vIdNext was: " + std::to_string(vIdNext) + " numVertices was: " + std::to_string(vertices_.size())).c_str());
    BOOST_ASSERT_MSG(vId < vertices_.size(), ("vId was: " + std::to_string(vId) + " numVertices was: " + std::to_string(vertices_.size())).c_str());
    BOOST_ASSERT_MSG(vId >= 0, ("vId was: " + std::to_string(vId) + " numVertices was: " + std::to_string(vertices_.size())).c_str());

    success &= getLineCoefficientsFromVertices(vertices_[vId], vertices_[vIdNext],
                                    lineCoefficients_[vId], vertexOrder_);
  }

  return success;

}

bool Polygon::resizePolygonTowardsInCenter(double lengthLineToLine) {
  if (vertices_.size() <= 1u || lengthLineToLine == 0.0) {
    return true;
  }

  if (vertices_.size() == 2u) {
    const Eigen::Vector2d middlePosition = 0.5*(vertices_[0] + vertices_[1]);
    const Eigen::Vector2d direction = (vertices_[0] - vertices_[1]);
    const double correctionLength = 0.5 * direction.norm() - lengthLineToLine;

    if (correctionLength<0.0) {
      return false;
    }

    const Eigen::Vector2d correction = direction.normalized()*correctionLength;

    vertices_[0] = middlePosition + correction;
    vertices_[1] = middlePosition - correction;

    return true;
  }

  VertexList resizedVertices(vertices_.size());
  for (unsigned int k=0u; k<vertices_.size(); ++k) {
    const auto kNext = robot_utils::intmod(k+1, vertices_.size());
    const auto kPrev = robot_utils::intmod(k-1, vertices_.size());

    BOOST_ASSERT_MSG(kNext < vertices_.size(), ("k was: " + std::to_string(k) + " kNext was: " + std::to_string(kNext) + " numVertices was: " + std::to_string(vertices_.size())).c_str());
    BOOST_ASSERT_MSG(kNext >= 0, ("k was: " + std::to_string(k) + " kNext was: " + std::to_string(kNext) + " numVertices was: " + std::to_string(vertices_.size())).c_str());
    BOOST_ASSERT_MSG(kPrev < vertices_.size(), ("k was: " + std::to_string(k) + " kPrev was: " + std::to_string(kPrev) + " numVertices was: " + std::to_string(vertices_.size())).c_str());
    BOOST_ASSERT_MSG(kPrev >= 0, ("k was: " + std::to_string(k) + " kPrev was: " + std::to_string(kPrev) + " numVertices was: " + std::to_string(vertices_.size())).c_str());

    // normalized vectors describing a triangle
    const Eigen::Vector2d d1 = (vertices_[kNext] - vertices_[k]).normalized();
    const Eigen::Vector2d d2 = (vertices_[kPrev] - vertices_[k]).normalized();
    const Eigen::Vector2d d = d1+d2;

    // determinant: sin(theta) = det(d1 d) / (|d1|*|d|) = sin(theta) = det(d1 d) / |d|
    // theta is the angle between d1 and d.
    const double det = d1.x()*d.y() - d1.y()*d.x();
    resizedVertices[k] = vertices_[k] + (lengthLineToLine/det) * d; // = vertices_[k] + (lengthLineToLine/sin(theta)) * d/|d|
  }

  // Reallocation.
  vertices_ = VertexList(resizedVertices.begin(), resizedVertices.end());
  return true;
}

bool Polygon::resizeVerticesTowardsCentroid(double percent) {
  bool success = true;

  if (vertices_.size() <= 1u || percent == 0.0) {
    return true;
  }

  Eigen::Vector2d center = Eigen::Vector2d::Zero();
  success &= computePolygonCentroid(center);
  const double oneMinusPercent = 1.0-percent;

  for (auto& vertex : vertices_) {
    vertex = center + (vertex - center)*oneMinusPercent;
  }

  return success;
}

bool Polygon::resizeAreaTriangleBased(double percent) {

  if (vertices_.size() <= 1u || percent == 0.0) {
    return true;
  }

  // Resize line towards middle point.
  else if (vertices_.size() == 2u) {
    return resizePolygonTowardsInCenter((vertices_[0] - vertices_[1]).norm() * percent * 0.5);
  }

  // Resize triangle towards incenter (along described in center radius).
  else if (vertices_.size() == 3u) {
    return resizePolygonTowardsInCenter(percent*computeInRadiusOfTriangle(vertices_[0u], vertices_[1u], vertices_[2u]));
  }


  bool success = true;
  if (vertexOrder_== VertexOrder::Undefined) {
    success &= reorderVertices(VertexOrder::CounterClockWise);
  }

  // Extract Adjacent triangles from support polygon and compute described inreadius.
  double averageIncircleRadius = 0.0;
  for (unsigned int id = 0u; id<vertices_.size(); ++id) {
    averageIncircleRadius +=
        computeInRadiusOfTriangle(
            vertices_[id],
            vertices_[robot_utils::intmod(id+1u, vertices_.size())],
            vertices_[robot_utils::intmod(id+2u, vertices_.size())]
        );
  }

  // Scale.
  averageIncircleRadius /= static_cast<double>(vertices_.size());

  // Resize polygon with the new safety margin.
  success &= resizePolygonTowardsInCenter(percent*averageIncircleRadius);

  return success;
}

void Polygon::resetVertices(unsigned int numOfVertices) {
  vertices_.clear();
  vertexOrder_ = robot_utils::geometry::VertexOrder::Undefined;
  lineCoefficients_.clear();

  if (numOfVertices>0u) {
    vertices_.resize(numOfVertices);
  }
}

void Polygon::print() const {
  int index = 0;
  for (const auto& v : vertices_) {
    std::cout << "v" << index << ": " << v.transpose() << std::endl;
    ++index;
  }
}

bool Polygon::doPolygonsIntersect(const Polygon& polygonA, const Polygon& polygonB) {
  Eigen::Vector2d rangeA = Eigen::Vector2d::Zero();
  Eigen::Vector2d rangeB = Eigen::Vector2d::Zero();
  Eigen::Vector2d normal;

  // Project on axis constructed from polygon A.
  for (const auto& lineCoefficients : polygonA.getLineCoefficients()) {
    normal = Eigen::Vector2d(lineCoefficients[0], lineCoefficients[1]).normalized();
    projectPolygonToRange(polygonA, normal, rangeA);
    projectPolygonToRange(polygonB, normal, rangeB);
    if (!doRangesOverlap(rangeA, rangeB)) {
      return false;
    }
  }

  // Project on axis constructed from polygon B.
  for (const auto& lineCoefficients : polygonB.getLineCoefficients()) {
    normal = Eigen::Vector2d(lineCoefficients[0], lineCoefficients[1]).normalized();
    projectPolygonToRange(polygonA, normal, rangeA);
    projectPolygonToRange(polygonB, normal, rangeB);
    if (!doRangesOverlap(rangeA, rangeB)) {
      return false;
    }
  }

  return true;
}

void Polygon::projectPolygonToRange(
    const Polygon& polygon,
    const Position2d& axisDirection,
    Eigen::Vector2d& range) {

  // Initialization.
  double min = axisDirection.dot(polygon.getVertices()[0]);
  double max = min;

  for (unsigned int k=1u; k<polygon.getVertices().size(); ++k) {
    const double p = axisDirection.dot(polygon.getVertices()[k]);
    if      (p < min) { min = p; }
    else if (p > max) { max = p; }
  }
  range << min, max;
}

bool Polygon::doRangesOverlap(
    const Position2d& rangeA,
    const Position2d& rangeB) {
  // Note: rangeA.x = min, rangeA.y = max

  // Bmin is in range A.
  if (rangeB.x() >= rangeA.x() && rangeB.x() <= rangeA.y()) {
    return true;
  }

  // Bmax is in range A.
  if (rangeB.y() >= rangeA.x() && rangeB.y() <= rangeA.y()) {
    return true;
  }

  // Amin is in range B.
  if (rangeA.x() >= rangeB.x() && rangeA.x() <= rangeB.y()) {
    return true;
  }

  // Amax is in range B.
  if (rangeA.y() >= rangeB.x() && rangeA.y() <= rangeB.y()) {
    return true;
  }

  return false;
}

bool Polygon::computePolygonCentroid(Eigen::Vector2d& center) const {
  center.setZero();
  for (const auto& vertex : vertices_) {
    center += vertex;
  }

  if (vertices_.size() > 0u) {
    center /= vertices_.size();
  } else { return false; }

  return true;
}

bool Polygon::computeClosestPointOnPolygon(Position2d& pointOnPolygon, const Position2d& point) const {

  // Find two vertices of the polygon that are closest to the point.
  Position2d p1; // closest point
  Position2d p2; // second closest point
  double minSqrtDinstPointToVertex = 1.0e100;

  for (const Position2d& vertex : vertices_) {
    const double sqrtDinstPointToVertex = (point-vertex).squaredNorm();

    if (sqrtDinstPointToVertex<minSqrtDinstPointToVertex) {
      minSqrtDinstPointToVertex = sqrtDinstPointToVertex;
      p2 = p1;
      p1 = vertex;
    }
  }

  // Find closest point to that line.
  const double distP1ToP2 = (p2-p1).norm();

  if (distP1ToP2==0.0) {
    std::cout << "[Polygon::computeClosestPointOnPolygon] Zero distance.\n";
    return false;
  }

  const Position2d n = (p2-p1)/distP1ToP2; // unit vector p1->p2.
  const Position2d pointToP1 = p1-point;
  const Position2d pointToLine = pointToP1 - (pointToP1.dot(n))*n;
  const Position2d pointOnLine = point + pointToLine;

  // Check if point on line is in polygon.
  if ((pointOnLine-p1).norm() > distP1ToP2 || (pointOnLine-p2).norm() > distP1ToP2) {
    pointOnPolygon = p1;
  } else {
    pointOnPolygon = pointOnLine;
  }

  return true;
}


} /* namespace robot_utils */
} /* namespace geometry */
