#pragma once

#include "Point.h"
#include <cstddef>
#include <vector>

inline constexpr int kMaxPolygonVertices = 30;
inline constexpr int kMinRandomPolygonVertices = 5;

inline bool polygonWithinVertexLimit(const std::vector<Point>& poly) {
    return poly.size() >= 3 && poly.size() <= static_cast<size_t>(kMaxPolygonVertices);
}

inline bool pairWithinVertexLimit(
    const std::vector<Point>& p0,
    const std::vector<Point>& p)
{
    return polygonWithinVertexLimit(p0) && polygonWithinVertexLimit(p);
}
