#pragma once
#include <vector>
#include "Point.h"
#include <random>

class HausdorffPolygon {
public:
    HausdorffPolygon() = default;
    explicit HausdorffPolygon(std::vector<Point> convex);

	// Многоугольник и его оболочка
    const std::vector<Point>& convex() const;
    const std::vector<Point>& convexHull() const;

    std::vector<Point>& convexHullMutable();

    // Центроид
    const Point& initialCentroid() const;
    double area() const;
    double convexHullArea() const;
    int reflexCount() const;

    double areaHullRatio() const;

private:
    std::vector<Point> m_convex;
    std::vector<Point> m_convex_hull;
    double m_area;
    double m_convex_hull_area;    
    Point m_initialCentroid;    
    std::vector<Point> convexHull(std::vector<Point>);
};