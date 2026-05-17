#include "metric_calculator.h"
#include "geometry.h"
#include <algorithm>
#include <cmath>

namespace {

int reflexCount(const std::vector<Point>& poly) {
    const int n = static_cast<int>(poly.size());
    int cnt = 0;
    for (int i = 0; i < n; ++i) {
        const Point& a = poly[static_cast<size_t>((i - 1 + n) % n)];
        const Point& b = poly[static_cast<size_t>(i)];
        const Point& c = poly[static_cast<size_t>((i + 1) % n)];
        if (geom::cross(a, b, c) < 0) ++cnt;
    }
    return cnt;
}

double scaleD(const std::vector<Point>& hull) {
    const double a = geom::polygonArea(hull);
    if (a > 1e-12) return std::sqrt(a);
    return std::max(geom::diameter(hull), 1e-6);
}

}  // namespace

PolygonMetrics computeMetrics(
    const std::vector<Point>& p0,
    const std::vector<Point>& p,
    const std::vector<PocketRecord>& pockets)
{
    PolygonMetrics m;
    const double areaP = geom::polygonArea(p);
    const double areaHull = geom::polygonArea(p0);
    m.n_hull = static_cast<int>(p0.size());
    m.area_ratio = (areaP > 1e-12) ? (areaHull / areaP) : 1.0;
    const double D = scaleD(p0);
    const double dMax = geom::maxIndentationToHull(p, p0);
    m.depth_rel = dMax / D;

    const double L = geom::perimeter(p0);
    double maxBridge = 0.0;
    double maxPocketW = 0.0;
    if (!pockets.empty()) {
        for (const auto& rec : pockets) {
            maxBridge = std::max(maxBridge, rec.bridgeChord);
            maxPocketW = std::max(maxPocketW, rec.mouthWidth);
        }
    } else {
        const int n = static_cast<int>(p0.size());
        for (int start = 0; start < n; ++start) {
            const Point& a = p0[static_cast<size_t>(start)];
            const Point& b = p0[static_cast<size_t>((start + 1) % n)];
            maxBridge = std::max(maxBridge, geom::dist(a, b));
            maxPocketW = std::max(maxPocketW, geom::dist(a, b));
        }
    }
    m.bridge_width_rel = (L > 1e-12) ? (maxBridge / L) : 0.0;
    m.pocket_width_rel = (D > 1e-12) ? (maxPocketW / D) : 0.0;

    double alphaMax = 0.0;
    const int pn = static_cast<int>(p.size());
    for (int i = 0; i < pn; ++i) {
        const Point& a = p[static_cast<size_t>((i - 1 + pn) % pn)];
        const Point& b = p[static_cast<size_t>(i)];
        const Point& c = p[static_cast<size_t>((i + 1) % pn)];
        const double cr = geom::cross(a, b, c);
        if (cr >= 0) continue;
        const double lab = geom::dist(a, b);
        const double lbc = geom::dist(b, c);
        if (lab < 1e-9 || lbc < 1e-9) continue;
        const double ux = (a.x - b.x) / lab;
        const double uy = (a.y - b.y) / lab;
        const double vx = (c.x - b.x) / lbc;
        const double vy = (c.y - b.y) / lbc;
        const double dot = std::max(-1.0, std::min(1.0, ux * vx + uy * vy));
        const double angle = std::acos(dot);
        constexpr double kPi = 3.14159265358979323846;
        alphaMax = std::max(alphaMax, kPi - angle);
    }
    m.alpha_proxy = alphaMax;
    m.reflex_count = reflexCount(p);
    return m;
}

PolygonMetrics computeMetrics(
    const std::vector<Point>& p0,
    const std::vector<Point>& p,
    int pocketChainK)
{
    const int n = static_cast<int>(p0.size());
    const int k = std::max(1, std::min(pocketChainK, n));
    std::vector<PocketRecord> pockets(1);
    pockets[0].spanEdges = k;
    pockets[0].bridgeChord = geom::dist(p0[0], p0[static_cast<size_t>(k % n)]);
    pockets[0].mouthWidth = pockets[0].bridgeChord;
    for (int start = 0; start < n; ++start) {
        const int end = (start + k) % n;
        const double chord = geom::dist(p0[static_cast<size_t>(start)], p0[static_cast<size_t>(end)]);
        if (chord > pockets[0].bridgeChord) {
            pockets[0].startIdx = start;
            pockets[0].bridgeChord = chord;
            pockets[0].mouthWidth = chord;
        }
    }
    return computeMetrics(p0, p, pockets);
}
