#include "HausdorffPolygon.h"
#include <cmath>

static double cross(const Point& a, const Point& b, const Point& c) {
    return (b.x - a.x) * (c.y - a.y) -
        (b.y - a.y) * (c.x - a.x);
}

static double polygonArea(const std::vector<Point>& p) {
    double a = 0.0;
    for (size_t i = 0; i < p.size(); ++i) {
        size_t j = (i + 1) % p.size();
        a += p[i].x * p[j].y - p[j].x * p[i].y;
    }
    return std::abs(a) * 0.5;
}

static Point computeCentroid(const std::vector<Point>& poly) {
    double A = 0, Cx = 0, Cy = 0;
    for (size_t i = 0; i < poly.size(); ++i) {
        size_t j = (i + 1) % poly.size();
        double c = poly[i].x * poly[j].y - poly[j].x * poly[i].y;
        A += c;
        Cx += (poly[i].x + poly[j].x) * c;
        Cy += (poly[i].y + poly[j].y) * c;
    }
    A *= 0.5;
    if (std::abs(A) < 1e-9) return { 0, 0 };
    return { Cx / (6 * A), Cy / (6 * A) };
}

HausdorffPolygon::HausdorffPolygon(std::vector<Point> convex)
    : m_convex(std::move(convex)),
    m_deformed(m_convex),
    m_initialCentroid(computeCentroid(m_convex))
{
}

const std::vector<Point>& HausdorffPolygon::convex() const {
    return m_convex;
}

const std::vector<Point>& HausdorffPolygon::deformed() const {
    return m_deformed;
}

std::vector<Point>& HausdorffPolygon::deformedMutable() {
    return m_deformed;
}

const Point& HausdorffPolygon::initialCentroid() const {
    return m_initialCentroid;
}

double HausdorffPolygon::area() const {
    return polygonArea(m_deformed);
}

double HausdorffPolygon::convexArea() const {
    return polygonArea(m_convex);
}

int HausdorffPolygon::reflexCount() const {
    int n = (int)m_deformed.size();
    int cnt = 0;
    for (int i = 0; i < n; ++i) {
        const Point& a = m_deformed[(i - 1 + n) % n];
        const Point& b = m_deformed[i];
        const Point& c = m_deformed[(i + 1) % n];
        if (cross(a, b, c) < 0)
            cnt++;
    }
    return cnt;
}

void HausdorffPolygon::resetToConvex() {
    m_deformed = m_convex;
}
