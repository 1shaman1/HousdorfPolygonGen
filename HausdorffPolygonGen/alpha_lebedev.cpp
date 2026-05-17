#include "alpha_lebedev.h"

#include "geometry.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;

bool pointInPolygon(const std::vector<Point>& poly, const Point& p) {
    const int n = static_cast<int>(poly.size());
    if (n < 3) return false;
    bool inside = false;
    for (int i = 0, j = n - 1; i < n; j = i++) {
        const Point& a = poly[static_cast<size_t>(i)];
        const Point& b = poly[static_cast<size_t>(j)];
        if (((a.y > p.y) != (b.y > p.y)) &&
            (p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y + 1e-30) + a.x)) {
            inside = !inside;
        }
    }
    return inside;
}

void collectOmega(
    const std::vector<Point>& poly,
    const Point& z,
    double tol,
    std::vector<Point>& omega)
{
    omega.clear();
    const int n = static_cast<int>(poly.size());
    double dMin = std::numeric_limits<double>::infinity();

    for (int i = 0; i < n; ++i) {
        const Point& a = poly[static_cast<size_t>(i)];
        const Point& b = poly[static_cast<size_t>((i + 1) % n)];
        const Point q = geom::segmentClosestPoint(z, a, b);
        dMin = std::min(dMin, geom::dist(z, q));
    }

    if (!std::isfinite(dMin)) return;

    auto addIfClose = [&](const Point& q) {
        if (geom::dist(z, q) <= dMin + tol) {
            for (const auto& w : omega) {
                if (geom::dist(w, q) < tol) return;
            }
            omega.push_back(q);
        }
    };

    for (int i = 0; i < n; ++i) {
        const Point& a = poly[static_cast<size_t>(i)];
        const Point& b = poly[static_cast<size_t>((i + 1) % n)];
        addIfClose(geom::segmentClosestPoint(z, a, b));
        addIfClose(a);
        addIfClose(b);
    }
}

double angleBetween(const Point& z, const Point& a, const Point& b) {
    const double ux = a.x - z.x;
    const double uy = a.y - z.y;
    const double vx = b.x - z.x;
    const double vy = b.y - z.y;
    const double lu = std::hypot(ux, uy);
    const double lv = std::hypot(vx, vy);
    if (lu < 1e-12 || lv < 1e-12) return 0.0;
    const double dot = std::max(-1.0, std::min(1.0, (ux * vx + uy * vy) / (lu * lv)));
    return std::acos(dot);
}

double alphaAtPoint(const std::vector<Point>& poly, const Point& z, double tol) {
    if (pointInPolygon(poly, z)) return 0.0;

    std::vector<Point> omega;
    collectOmega(poly, z, tol, omega);
    if (omega.size() < 2) return 0.0;

    double best = 0.0;
    for (size_t i = 0; i < omega.size(); ++i) {
        for (size_t j = i + 1; j < omega.size(); ++j) {
            best = std::max(best, angleBetween(z, omega[i], omega[j]));
        }
    }
    return best;
}

void appendReflexBisectorSamples(const std::vector<Point>& poly, std::vector<Point>& candidates) {
    const int n = static_cast<int>(poly.size());
    const double scale = std::max(geom::diameter(poly), 1e-6);
    for (int i = 0; i < n; ++i) {
        const Point& a = poly[static_cast<size_t>((i - 1 + n) % n)];
        const Point& b = poly[static_cast<size_t>(i)];
        const Point& c = poly[static_cast<size_t>((i + 1) % n)];
        if (geom::cross(a, b, c) >= 0) continue;

        const double lab = geom::dist(a, b);
        const double lbc = geom::dist(c, b);
        if (lab < 1e-9 || lbc < 1e-9) continue;

        Point u{(a.x - b.x) / lab + (c.x - b.x) / lbc, (a.y - b.y) / lab + (c.y - b.y) / lbc};
        const double lu = std::hypot(u.x, u.y);
        if (lu < 1e-12) continue;
        u.x /= lu;
        u.y /= lu;

        for (double t : {0.05, 0.15, 0.35, 0.7, 1.2, 2.0}) {
            candidates.push_back({b.x + u.x * t * scale, b.y + u.y * t * scale});
        }
    }
}

void appendVertexPairBisectors(const std::vector<Point>& poly, std::vector<Point>& candidates) {
    const int n = static_cast<int>(poly.size());
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            const Point& a = poly[static_cast<size_t>(i)];
            const Point& b = poly[static_cast<size_t>(j)];
            const Point mid{(a.x + b.x) * 0.5, (a.y + b.y) * 0.5};
            Point e{b.x - a.x, b.y - a.y};
            const double le = std::hypot(e.x, e.y);
            if (le < 1e-9) continue;
            Point nrm{-e.y / le, e.x / le};
            const double half = le * 0.5;
            for (int s = -1; s <= 1; s += 2) {
                candidates.push_back({mid.x + nrm.x * half * s, mid.y + nrm.y * half * s});
            }
        }
    }
}

void appendExteriorGrid(const std::vector<Point>& poly, std::vector<Point>& candidates, int gridN) {
    double xmin = poly[0].x, xmax = poly[0].x, ymin = poly[0].y, ymax = poly[0].y;
    for (const auto& p : poly) {
        xmin = std::min(xmin, p.x);
        xmax = std::max(xmax, p.x);
        ymin = std::min(ymin, p.y);
        ymax = std::max(ymax, p.y);
    }
    const double margin = 0.25 * std::max(geom::diameter(poly), 1e-6);
    xmin -= margin;
    xmax += margin;
    ymin -= margin;
    ymax += margin;

    for (int iy = 0; iy <= gridN; ++iy) {
        for (int ix = 0; ix <= gridN; ++ix) {
            const double tx = static_cast<double>(ix) / gridN;
            const double ty = static_cast<double>(iy) / gridN;
            const Point z{xmin + (xmax - xmin) * tx, ymin + (ymax - ymin) * ty};
            if (!pointInPolygon(poly, z)) candidates.push_back(z);
        }
    }
}

}  // namespace

double computeAlphaLebedev(const std::vector<Point>& poly) {
    if (poly.size() < 3) return 0.0;
    if (geom::isConvex(poly)) return 0.0;

    const double tol = 1e-6 * std::max(geom::diameter(poly), 1.0);

    std::vector<Point> candidates;
    candidates.reserve(1024);
    appendReflexBisectorSamples(poly, candidates);
    appendVertexPairBisectors(poly, candidates);
    appendExteriorGrid(poly, candidates, 28);

    double best = 0.0;
    for (const auto& z : candidates) {
        best = std::max(best, alphaAtPoint(poly, z, tol));
        if (best >= kPi - 1e-9) return kPi;
    }

    return std::min(best, kPi);
}
