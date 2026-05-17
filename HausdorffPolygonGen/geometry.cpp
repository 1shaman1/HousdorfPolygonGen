#include "geometry.h"
#include <algorithm>
#include <cmath>

namespace geom {

double cross(const Point& a, const Point& b, const Point& c) {
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

double distSq(const Point& a, const Point& b) {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    return dx * dx + dy * dy;
}

double dist(const Point& a, const Point& b) {
    return std::sqrt(distSq(a, b));
}

double polygonArea(const std::vector<Point>& p) {
    if (p.size() < 3) return 0.0;
    double a = 0.0;
    for (size_t i = 0; i < p.size(); ++i) {
        size_t j = (i + 1) % p.size();
        a += p[i].x * p[j].y - p[j].x * p[i].y;
    }
    return std::abs(a) * 0.5;
}

double perimeter(const std::vector<Point>& p) {
    if (p.size() < 2) return 0.0;
    double len = 0.0;
    for (size_t i = 0; i < p.size(); ++i) {
        len += dist(p[i], p[(i + 1) % p.size()]);
    }
    return len;
}

double diameter(const std::vector<Point>& p) {
    double best = 0.0;
    for (size_t i = 0; i < p.size(); ++i) {
        for (size_t j = i + 1; j < p.size(); ++j) {
            best = std::max(best, dist(p[i], p[j]));
        }
    }
    return best;
}

std::vector<Point> convexHull(std::vector<Point> pts) {
    if (pts.size() <= 1) return pts;
    std::sort(pts.begin(), pts.end(), [](const Point& a, const Point& b) {
        if (a.x == b.x) return a.y < b.y;
        return a.x < b.x;
    });

    std::vector<Point> lower, upper;
    for (const auto& p : pts) {
        while (lower.size() >= 2 &&
               cross(lower[lower.size() - 2], lower.back(), p) <= 0)
            lower.pop_back();
        lower.push_back(p);
    }
    for (int i = static_cast<int>(pts.size()) - 1; i >= 0; --i) {
        const auto& p = pts[static_cast<size_t>(i)];
        while (upper.size() >= 2 &&
               cross(upper[upper.size() - 2], upper.back(), p) <= 0)
            upper.pop_back();
        upper.push_back(p);
    }
    lower.pop_back();
    upper.pop_back();
    lower.insert(lower.end(), upper.begin(), upper.end());
    return lower;
}

bool isConvex(const std::vector<Point>& p) {
    const int n = static_cast<int>(p.size());
    if (n < 4) return true;
    int sign = 0;
    for (int i = 0; i < n; i++) {
        const Point& a = p[static_cast<size_t>(i)];
        const Point& b = p[static_cast<size_t>((i + 1) % n)];
        const Point& c = p[static_cast<size_t>((i + 2) % n)];
        const double cr = cross(a, b, c);
        if (cr == 0) continue;
        if (sign == 0)
            sign = (cr > 0) ? 1 : -1;
        else if ((cr > 0 && sign < 0) || (cr < 0 && sign > 0))
            return false;
    }
    return true;
}

bool segmentsIntersect(const Point& a1, const Point& a2, const Point& b1, const Point& b2) {
    auto orient = [](const Point& p, const Point& q, const Point& r) {
        return (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
    };
    auto onSeg = [](const Point& p, const Point& q, const Point& r) {
        return q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) &&
               q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y);
    };

    const double o1 = orient(a1, a2, b1);
    const double o2 = orient(a1, a2, b2);
    const double o3 = orient(b1, b2, a1);
    const double o4 = orient(b1, b2, a2);

    if (o1 * o2 < 0 && o3 * o4 < 0) return true;
    if (o1 == 0 && onSeg(a1, b1, a2)) return true;
    if (o2 == 0 && onSeg(a1, b2, a2)) return true;
    if (o3 == 0 && onSeg(b1, a1, b2)) return true;
    if (o4 == 0 && onSeg(b1, a2, b2)) return true;
    return false;
}

bool isSimple(const std::vector<Point>& poly) {
    const int n = static_cast<int>(poly.size());
    if (n < 3) return false;
    for (int i = 0; i < n; ++i) {
        const Point a1 = poly[static_cast<size_t>(i)];
        const Point a2 = poly[static_cast<size_t>((i + 1) % n)];
        for (int j = i + 1; j < n; ++j) {
            if (std::abs(i - j) <= 1 || (i == 0 && j == n - 1)) continue;
            const Point b1 = poly[static_cast<size_t>(j)];
            const Point b2 = poly[static_cast<size_t>((j + 1) % n)];
            if (segmentsIntersect(a1, a2, b1, b2)) return false;
        }
    }
    return true;
}

Point segmentClosestPoint(const Point& p, const Point& a, const Point& b) {
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double len2 = dx * dx + dy * dy;
    if (len2 < 1e-18) return a;
    double t = ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2;
    t = std::max(0.0, std::min(1.0, t));
    return {a.x + t * dx, a.y + t * dy};
}

double pointToSegmentDist(const Point& p, const Point& a, const Point& b) {
    const Point q = segmentClosestPoint(p, a, b);
    return dist(p, q);
}

bool hullsApproximatelyEqual(
    const std::vector<Point>& a,
    const std::vector<Point>& b,
    double relTol)
{
    if (a.size() != b.size()) return false;
    const double scale = std::max(geom::diameter(a), 1e-6);
    const double tol = relTol * scale;
    for (size_t i = 0; i < a.size(); ++i) {
        bool found = false;
        for (const auto& q : b) {
            if (dist(a[i], q) <= tol) {
                found = true;
                break;
            }
        }
        if (!found) return false;
    }
    return true;
}

double maxIndentationToHull(const std::vector<Point>& p, const std::vector<Point>& hull) {
    const int hn = static_cast<int>(hull.size());
    double best = 0.0;
    for (const auto& v : p) {
        double toBoundary = pointToSegmentDist(
            v, hull[0], hull[static_cast<size_t>((hn > 1) ? 1 : 0)]);
        for (int i = 0; i < hn; ++i) {
            const Point& a = hull[static_cast<size_t>(i)];
            const Point& b = hull[static_cast<size_t>((i + 1) % hn)];
            toBoundary = std::min(toBoundary, pointToSegmentDist(v, a, b));
        }
        best = std::max(best, toBoundary);
    }
    return best;
}

}  // namespace geom
