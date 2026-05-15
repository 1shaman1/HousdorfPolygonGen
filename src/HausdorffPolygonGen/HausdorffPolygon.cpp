#include "HausdorffPolygon.h"
#include <algorithm>
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
    m_initialCentroid(computeCentroid(m_convex))        
{
    m_convex_hull = convexHull(m_convex);
    m_area = polygonArea(m_convex);
    m_convex_hull_area = polygonArea(m_convex_hull);
}

const std::vector<Point>& HausdorffPolygon::convex() const {
    return m_convex;
}

const std::vector<Point>& HausdorffPolygon::convexHull() const {
    return m_convex_hull;
}

std::vector<Point>& HausdorffPolygon::convexHullMutable() {
    return m_convex_hull;
}

const Point& HausdorffPolygon::initialCentroid() const {
    return m_initialCentroid;
}

double HausdorffPolygon::area() const {
    return m_area;
}

double HausdorffPolygon::convexHullArea() const {
    return m_convex_hull_area;
}

double HausdorffPolygon::areaHullRatio() const {
    return m_convex_hull_area / m_area ;
}

int HausdorffPolygon::reflexCount() const {
    int n = (int)m_convex.size();
    int cnt = 0;
    for (int i = 0; i < n; ++i) {
        const Point& a = m_convex[(i - 1 + n) % n];
        const Point& b = m_convex[i];
        const Point& c = m_convex[(i + 1) % n];
        if (cross(a, b, c) < 0)
            cnt++;
    }
    return cnt;
}

/// @brief Построение выпуклого многоугольника по алг. Грэхема
/// @param points 
/// @return 
std::vector<Point> HausdorffPolygon::convexHull(std::vector<Point> points)
{
    int n = points.size();
    if (n < 3)
        return points;

    // 1. ищем самую нижнюю точку
    int pivot = 0;
    for (int i = 1; i < n; i++)
    {
        if (points[i].y < points[pivot].y ||
           (points[i].y == points[pivot].y &&
            points[i].x < points[pivot].x))
            pivot = i;
    }

    std::swap(points[0], points[pivot]);
    Point p0 = points[0];

    // 2. сортировка по полярному углу
    std::sort(points.begin() + 1, points.end(),
        [&](const Point& a, const Point& b)
        {
            double c = cross(p0, a, b);

            if (c == 0)
                return (p0.x - a.x) * (p0.x - a.x) + (p0.y - a.y) * (p0.y - a.y) < 
                    (p0.x - b.x) * (p0.x - b.x) + (p0.y - b.y) * (p0.y - b.y);

            return c > 0;
        });

    // 3. построение оболочки
    std::vector<Point> hull;
    hull.push_back(points[0]);
    hull.push_back(points[1]);

    for (int i = 2; i < n; i++)
    {
        while (hull.size() >= 2)
        {
            Point b = hull.back();
            Point a = hull[hull.size() - 2];

            if (cross(a, b, points[i]) > 0)
                break;

            hull.pop_back();
        }

        hull.push_back(points[i]);
    }

    return hull;
}

