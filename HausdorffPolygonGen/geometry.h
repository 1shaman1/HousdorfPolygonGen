#pragma once

#include "Point.h"
#include <vector>
#include <cmath>

namespace geom {

double cross(const Point& a, const Point& b, const Point& c);
double distSq(const Point& a, const Point& b);
double dist(const Point& a, const Point& b);
double polygonArea(const std::vector<Point>& p);
double perimeter(const std::vector<Point>& p);
double diameter(const std::vector<Point>& p);
std::vector<Point> convexHull(std::vector<Point> pts);
bool isConvex(const std::vector<Point>& p);
bool isSimple(const std::vector<Point>& poly);
bool segmentsIntersect(const Point& a, const Point& b, const Point& c, const Point& d);
Point segmentClosestPoint(const Point& p, const Point& a, const Point& b);
double pointToSegmentDist(const Point& p, const Point& a, const Point& b);
double maxIndentationToHull(const std::vector<Point>& p, const std::vector<Point>& hull);
bool hullsApproximatelyEqual(
    const std::vector<Point>& a,
    const std::vector<Point>& b,
    double relTol = 1e-4);

}  // namespace geom
