/*!
* @file     GeometryTest.cpp
* @author   Dario Bellicoso
* @date     January 27, 2017
* @brief
*/

// gtest
#include <gtest/gtest.h>

// robot utils
#include <robot_utils/geometry/Polygon.hpp>
#include <robot_utils/geometry/Line.hpp>
#include <robot_utils/geometry/Triangle.hpp>
#include <robot_utils/geometry/Tetragon.hpp>
#include <robot_utils/math/math.hpp>


TEST(Geometry, GetLineVertices) {
  using namespace robot_utils;
  using namespace robot_utils::geometry;

  const Position2d v1(0.0, 0.0);
  const Position2d v2(5.0, 0.0);
  const Line::VertexList vertices = {v1, v2};

  robot_utils::geometry::Line line(vertices, VertexOrder::ClockWise);

  const Polygon::VertexList lineVertices = line.getVertices();

  EXPECT_TRUE(lineVertices[0].isApprox(vertices[0]));
  EXPECT_TRUE(lineVertices[1].isApprox(vertices[1]));
}

TEST(Geometry, IsPointOnLine) {
  using namespace robot_utils;
  using namespace robot_utils::geometry;

  const double length = 5.0;

  const Position2d v1(0.0,    0.0);
  const Position2d v2(length, 0.0);
  const Line::VertexList vertices = {v1, v2};

  robot_utils::geometry::Line line(vertices, VertexOrder::ClockWise);
  line.updateLineCoefficients();

  const Position2d pOn(length/2.0, 0.0);
  const Position2d pOut(length/2.0, length);

  EXPECT_TRUE(line.isPointInPolygon(pOn));
  EXPECT_TRUE(!line.isPointInPolygon(pOut));
}

TEST(Geometry, GetLineIntersection) {
  using namespace robot_utils;
  using namespace robot_utils::geometry;

  const Position2d l1A(0.0, 0.0);
  const Position2d l1B(2.0, 0.0);

  const Position2d l2A(1.0,  1.0);
  const Position2d l2B(1.0, -1.0);

  const Line::VertexList l1Vertices = {l1A, l1B};
  const Line::VertexList l2Vertices = {l2A, l2B};

  robot_utils::geometry::Line line1(l1Vertices, VertexOrder::ClockWise);
  robot_utils::geometry::Line line2(l2Vertices, VertexOrder::ClockWise);

  Position2d intersection;
  line1.findIntersection(line2, intersection);

  const Position2d expectedIntersection(1.0, 0.0);

  EXPECT_TRUE(intersection.isApprox(expectedIntersection));
}


TEST(Geometry, DoLinesIntersect) {
  using namespace robot_utils;
  using namespace robot_utils::geometry;

  const Position2d l1A(0.0, 0.0);
  const Position2d l1B(2.0, 0.0);

  const Position2d l2A(1.0,  0.0);
  const Position2d l2B(4.0, 0.0);

  const Line::VertexList l1Vertices = {l1A, l1B};
  const Line::VertexList l2Vertices = {l2A, l2B};

  robot_utils::geometry::Line line1(l1Vertices, VertexOrder::ClockWise);
  robot_utils::geometry::Line line2(l2Vertices, VertexOrder::ClockWise);

  Position2d intersection;
  const bool doLinesIntersect = Line::findIntersection(line1, line2, intersection);

  EXPECT_FALSE(doLinesIntersect);
}


TEST(Geometry, GetLineArea) {
  using namespace robot_utils;
  using namespace robot_utils::geometry;

  const Position2d v1(0.0, 0.0);
  const Position2d v2(5.0, 0.0);
  const Line::VertexList vertices = {v1, v2};

  robot_utils::geometry::Line line(vertices, VertexOrder::ClockWise);

  const double lineArea = line.getArea();

  EXPECT_TRUE(lineArea == 0.0);
}

TEST(Geometry, GetLineNormalClockwise) {
  using namespace robot_utils;
  using namespace robot_utils::geometry;

  const Position2d v1(0.0, 0.0);
  const Position2d v2(5.0, 0.0);
  const Line::VertexList vertices = {v1, v2};

  robot_utils::geometry::Line line(vertices, VertexOrder::ClockWise);

  line.updateLineCoefficients();

  const Polygon::LineCoefficientList& lineCoefficients = line.getLineCoefficients();

  const Eigen::Vector2d normal(lineCoefficients[0][0], lineCoefficients[0][1]);

  Eigen::Vector2d expectedNormal(0.0, -1.0);

  EXPECT_TRUE(normal.isApprox(expectedNormal));
}

TEST(Geometry, GetLineNormalCounterClockwise) {
  using namespace robot_utils;
  using namespace robot_utils::geometry;

  const Position2d v1(0.0, 0.0);
  const Position2d v2(5.0, 0.0);
  const Line::VertexList vertices = {v1, v2};

  robot_utils::geometry::Line line(vertices, VertexOrder::CounterClockWise);

  line.updateLineCoefficients();

  const Polygon::LineCoefficientList& lineCoefficients = line.getLineCoefficients();
  const Eigen::Vector2d normal(lineCoefficients[0][0], lineCoefficients[0][1]);

  const Eigen::Vector2d expectedNormal(0.0, 1.0);

  EXPECT_TRUE(normal.isApprox(expectedNormal));
}

TEST(Geometry, GetTriangleArea) {
  using namespace robot_utils;
  using namespace robot_utils::geometry;

  const double length = 5.0;

  const Position2d v1(0.0, 0.0);
  const Position2d v2(length, 0.0);
  const Position2d v3(0.0, length);
  const Polygon::VertexList vertices = {v1, v2, v3};

  robot_utils::geometry::Triangle triangle(vertices, VertexOrder::CounterClockWise);

  const double area = triangle.getArea();
  const double expectedArea = length*length/2.0;

  EXPECT_TRUE(area == expectedArea);
}

TEST(Geometry, GetTriangleNormalVectors) {
  using namespace robot_utils;
  using namespace robot_utils::geometry;

  const double length = 5.0;

  const Position2d v1(0.0,    0.0);
  const Position2d v2(length, 0.0);
  const Position2d v3(0.0,    length);
  const Polygon::VertexList vertices = {v1, v2, v3};

  robot_utils::geometry::Triangle triangle(vertices, VertexOrder::CounterClockWise);
  triangle.updateLineCoefficients();

  const Polygon::LineCoefficientList& lineCoefficients = triangle.getLineCoefficients();

  const Eigen::Vector2d n0(lineCoefficients[0][0], lineCoefficients[0][1]);
  const Eigen::Vector2d n1(lineCoefficients[1][0], lineCoefficients[1][1]);
  const Eigen::Vector2d n2(lineCoefficients[2][0], lineCoefficients[2][1]);

  const Eigen::Vector2d expectedNormal0(0.0, 1.0);
  const Eigen::Vector2d expectedNormal1(-std::cos(M_PI_4), -std::sin(M_PI_4));
  const Eigen::Vector2d expectedNormal2(1.0, 0.0);

  EXPECT_TRUE(n0.isApprox(expectedNormal0));
  EXPECT_TRUE(n1.isApprox(expectedNormal1));
  EXPECT_TRUE(n2.isApprox(expectedNormal2));
}

TEST(Geometry, IsPointInTriangle) {
  using namespace robot_utils;
  using namespace robot_utils::geometry;

  const double length = 5.0;

  const Position2d v1(0.0,    0.0);
  const Position2d v2(length, 0.0);
  const Position2d v3(0.0,    length);
  const Polygon::VertexList vertices = {v1, v2, v3};

  robot_utils::geometry::Triangle triangle(vertices, VertexOrder::CounterClockWise);
  triangle.updateLineCoefficients();

  const Position2d pIn(length/4.0, length/4.0);
  const Position2d pOut(2.0*length, 2.0*length);

  EXPECT_TRUE(triangle.isPointInPolygon(pIn));
  EXPECT_TRUE(!triangle.isPointInPolygon(pOut));
}

double doubleRand(double min = 1e-4, double max = 1.0) {
    const double f = (double)rand() / RAND_MAX;
    return (min + f * (max - min));
}

TEST(Geometry, TriangleReorderClockWise) {
  using namespace robot_utils;
  using namespace robot_utils::geometry;

  const double length = 5.0;

  const Eigen::Vector2d d1(0.0, 1.0);
  const Eigen::Vector2d d2(std::cos(M_PI_4), -std::sin(M_PI_4));
  const Eigen::Vector2d d3(-std::cos(M_PI_4), -std::sin(M_PI_4));

  const Position2d v1 = doubleRand()*d1;
  const Position2d v2 = doubleRand()*d2;
  const Position2d v3 = doubleRand()*d3;
  const Polygon::VertexList vertices = {v1, v2, v3};

  robot_utils::geometry::Triangle triangle(vertices, VertexOrder::CounterClockWise);

  triangle.reorderVertices(VertexOrder::ClockWise);
  const double negArea = triangle.getArea();

  triangle.reorderVertices(VertexOrder::CounterClockWise);
  const double posArea = triangle.getArea();

  EXPECT_TRUE(posArea > 0.0);
  EXPECT_TRUE(negArea < 0.0);
}

TEST(Geometry, SquareArea) {
  using namespace robot_utils;
  using namespace robot_utils::geometry;

  const double length = 5.0;

  const Eigen::Vector2d v1(0.0, 0.0);
  const Eigen::Vector2d v2(length, 0.0);
  const Eigen::Vector2d v3(length, length);
  const Eigen::Vector2d v4(0.0, length);

  const Polygon::VertexList vertices = {v1, v2, v3, v4};

  robot_utils::geometry::Tetragon square(vertices, VertexOrder::CounterClockWise);
  const double area = square.getArea();
  const double expectedArea = length*length;


  EXPECT_TRUE(area == expectedArea);
}

TEST(Geometry, SquareAreaResized) {
  using namespace robot_utils;
  using namespace robot_utils::geometry;

  const double length = 5.0;

  const Eigen::Vector2d v1(0.0, 0.0);
  const Eigen::Vector2d v2(length, 0.0);
  const Eigen::Vector2d v3(length, length);
  const Eigen::Vector2d v4(0.0, length);

  const Polygon::VertexList vertices = {v1, v2, v3, v4};

  robot_utils::geometry::Tetragon square(vertices, VertexOrder::CounterClockWise);
  const double area = square.getArea();
  const double expectedArea = length*length;

  EXPECT_TRUE(area == expectedArea);

  square.resizePolygonTowardsInCenter(1.0);

  double resizedArea = square.getArea();
  double expectedResizedArea = 9.0;

  EXPECT_TRUE(resizedArea == expectedResizedArea);

  // Restore area to original value.
  square.resizePolygonTowardsInCenter(-1.0);
  // Increase area again.
  square.resizePolygonTowardsInCenter(-1.0);

  resizedArea = square.getArea();
  expectedResizedArea = 49.0;

  EXPECT_TRUE(resizedArea == expectedResizedArea);

}

TEST(Geometry, ModulusTest) {
  const int a = 4;
  const int b = 7;
  EXPECT_TRUE(robot_utils::intmodRange(a, a, b) == 0);
  EXPECT_TRUE(robot_utils::intmodRange(a+1, a, b) == 1);
  EXPECT_TRUE(robot_utils::intmodRange(b+1, a, b) == 1);
  EXPECT_TRUE(robot_utils::intmodRange(b+2*(b-a), a, b) == 0);
}

TEST(Geometry, IntModTest) {
  const int range = 3;

  EXPECT_TRUE(robot_utils::intmod(range, range) == 0);
  EXPECT_TRUE(robot_utils::intmod(range+1, range) == 1);
  EXPECT_TRUE(robot_utils::intmod(0, range) == 0);
  EXPECT_TRUE(robot_utils::intmod(-1, range) == 2);
}

TEST(Geometry, IsLess) {
  const robot_utils::geometry::Position2d p1(0.0, 1.0);
  const robot_utils::geometry::Position2d p2(4.0, 1.0);
  const robot_utils::geometry::Position2d c1(2.0, 0.0);
  const robot_utils::geometry::Position2d c2(2.0, 2.0);

  EXPECT_TRUE(!robot_utils::geometry::less(p1, p2, c1));
  EXPECT_TRUE(robot_utils::geometry::less(p1, p2, c2));
}

TEST(Geometry, ProjectPolygonToAxis) {
  robot_utils::geometry::Polygon polygonA({
    robot_utils::geometry::Position2d(0.0, 0.0),
    robot_utils::geometry::Position2d(1.0, 0.0),
    robot_utils::geometry::Position2d(1.0, 1.0),
    robot_utils::geometry::Position2d(0.0, 1.0)});
  polygonA.reorderVertices(robot_utils::geometry::VertexOrder::CounterClockWise);
  polygonA.updateLineCoefficients();

  robot_utils::geometry::Polygon polygonB({
    robot_utils::geometry::Position2d(2.0 + 0.0, 0.0),
    robot_utils::geometry::Position2d(2.0 + 1.0, 0.0),
    robot_utils::geometry::Position2d(2.0 + 1.0, 1.0),
    robot_utils::geometry::Position2d(2.0 + 0.0, 1.0)});
  polygonB.reorderVertices(robot_utils::geometry::VertexOrder::CounterClockWise);
  polygonB.updateLineCoefficients();

  Eigen::Vector2d verticalAxis(0.0, 1.0);
  Eigen::Vector2d horizontalAxis(1.0, 0.0);
  Eigen::Vector2d range;

  robot_utils::geometry::Polygon::projectPolygonToRange(polygonA, verticalAxis, range);
  EXPECT_TRUE(range[0] == 0.0); EXPECT_TRUE(range[1] == 1.0);

  robot_utils::geometry::Polygon::projectPolygonToRange(polygonB, verticalAxis, range);
  EXPECT_TRUE(range[0] == 0.0); EXPECT_TRUE(range[1] == 1.0);

  robot_utils::geometry::Polygon::projectPolygonToRange(polygonA, horizontalAxis, range);
  EXPECT_TRUE(range[0] == 0.0); EXPECT_TRUE(range[1] == 1.0);

  robot_utils::geometry::Polygon::projectPolygonToRange(polygonB, horizontalAxis, range);
  EXPECT_TRUE(range[0] == 2.0); EXPECT_TRUE(range[1] == 3.0);
}

TEST(Geometry, RangeOverlap) {
  const Eigen::Vector2d rangeA(0.0, 2.0);
  const Eigen::Vector2d rangeB(1.0, 3.0);
  const Eigen::Vector2d rangeC(3.0, 4.0);
  const Eigen::Vector2d rangeD(-5.0, -1.0);
  const Eigen::Vector2d rangeE(-1.0, 3.0);
  const Eigen::Vector2d rangeF(0.5, 1.5);

  EXPECT_TRUE(robot_utils::geometry::Polygon::doRangesOverlap(rangeA, rangeB));
  EXPECT_FALSE(robot_utils::geometry::Polygon::doRangesOverlap(rangeA, rangeC));
  EXPECT_FALSE(robot_utils::geometry::Polygon::doRangesOverlap(rangeA, rangeD));
  EXPECT_TRUE(robot_utils::geometry::Polygon::doRangesOverlap(rangeA, rangeE));
  EXPECT_TRUE(robot_utils::geometry::Polygon::doRangesOverlap(rangeA, rangeF));
}

TEST(Geometry, DoPolygonsIntersec) {
  robot_utils::geometry::Polygon polygonA({
    robot_utils::geometry::Position2d(0.0, 0.0),
    robot_utils::geometry::Position2d(1.0, 0.0),
    robot_utils::geometry::Position2d(1.0, 1.0),
    robot_utils::geometry::Position2d(0.0, 1.0)});

  robot_utils::geometry::Polygon polygonB({
    robot_utils::geometry::Position2d(0.25 + 0.0, 0.0),
    robot_utils::geometry::Position2d(0.25 + 1.0, 0.0),
    robot_utils::geometry::Position2d(0.25 + 1.0, 1.0),
    robot_utils::geometry::Position2d(0.25 + 0.0, 1.0)});

  robot_utils::geometry::Polygon polygonC({
    robot_utils::geometry::Position2d(2.0 + 0.0, 0.0),
    robot_utils::geometry::Position2d(2.0 + 1.0, 0.0),
    robot_utils::geometry::Position2d(2.0 + 1.0, 1.0),
    robot_utils::geometry::Position2d(2.0 + 0.0, 1.0)});

  robot_utils::geometry::Polygon triangleA({
    robot_utils::geometry::Position2d(0.0, 0.0),
    robot_utils::geometry::Position2d(1.0, 0.0),
    robot_utils::geometry::Position2d(1.0, 1.0)});

  robot_utils::geometry::Polygon triangleB({
    robot_utils::geometry::Position2d(0.0, 0.0),
    robot_utils::geometry::Position2d(1.0, 1.0),
    robot_utils::geometry::Position2d(0.0, 1.0)});

  robot_utils::geometry::Polygon triangleC({
    robot_utils::geometry::Position2d(0.4 + 0.0, 0.0),
    robot_utils::geometry::Position2d(0.4 + 1.0, 0.0),
    robot_utils::geometry::Position2d(0.4 + 1.0, 1.0)});

  robot_utils::geometry::Polygon triangleD({
    robot_utils::geometry::Position2d(0.0, -0.5),
    robot_utils::geometry::Position2d(1.2, 0.5),
    robot_utils::geometry::Position2d(0.8, 0.9)});

  polygonA.reorderVertices(robot_utils::geometry::VertexOrder::CounterClockWise);
  polygonA.updateLineCoefficients();

  polygonB.reorderVertices(robot_utils::geometry::VertexOrder::CounterClockWise);
  polygonB.updateLineCoefficients();

  polygonC.reorderVertices(robot_utils::geometry::VertexOrder::CounterClockWise);
  polygonC.updateLineCoefficients();

  triangleA.reorderVertices(robot_utils::geometry::VertexOrder::CounterClockWise);
  triangleA.updateLineCoefficients();

  triangleB.reorderVertices(robot_utils::geometry::VertexOrder::CounterClockWise);
  triangleB.updateLineCoefficients();

  triangleC.reorderVertices(robot_utils::geometry::VertexOrder::CounterClockWise);
  triangleC.updateLineCoefficients();

  triangleD.reorderVertices(robot_utils::geometry::VertexOrder::CounterClockWise);
  triangleD.updateLineCoefficients();

  EXPECT_TRUE(robot_utils::geometry::Polygon::doPolygonsIntersect(triangleA, triangleB));
  EXPECT_FALSE(robot_utils::geometry::Polygon::doPolygonsIntersect(triangleB, triangleC));
  EXPECT_TRUE(robot_utils::geometry::Polygon::doPolygonsIntersect(polygonA, polygonB));
  EXPECT_FALSE(robot_utils::geometry::Polygon::doPolygonsIntersect(polygonB, polygonC));
  EXPECT_FALSE(robot_utils::geometry::Polygon::doPolygonsIntersect(polygonA, polygonC));

  EXPECT_TRUE(robot_utils::geometry::Polygon::doPolygonsIntersect(polygonA, polygonA));
  EXPECT_TRUE(robot_utils::geometry::Polygon::doPolygonsIntersect(triangleA, triangleD));
}

