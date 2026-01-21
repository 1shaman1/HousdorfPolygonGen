#pragma once
#include <vector>
#include "Point.h"

class HausdorffPolygon {
public:
    HausdorffPolygon() = default;
    explicit HausdorffPolygon(std::vector<Point> convex);

    // доступ к состояниям
    const std::vector<Point>& convex() const;
    const std::vector<Point>& deformed() const;

    std::vector<Point>& deformedMutable();

    // геометрия
    const Point& initialCentroid() const;
    double area() const;
    double convexArea() const;
    int reflexCount() const;

    // служебное
    void resetToConvex();   // откат деформаций

private:
    std::vector<Point> m_convex;
    std::vector<Point> m_deformed;
    Point m_initialCentroid;
};