#include "random_pocket_builder.h"

#include "geometry.h"
#include "metric_calculator.h"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;

double scaleD(const std::vector<Point>& hull) {
    const double a = geom::polygonArea(hull);
    if (a > 1e-12) return std::sqrt(a);
    return std::max(geom::diameter(hull), 1e-6);
}

int randomDentCount(std::mt19937& rng) {
    std::uniform_int_distribution<int> d(1, 4);
    return d(rng);
}

std::vector<int> pickRandomEdgeStarts(int n, int dentCount, std::mt19937& rng) {
    std::vector<int> edges(n);
    std::iota(edges.begin(), edges.end(), 0);
    std::shuffle(edges.begin(), edges.end(), rng);
    dentCount = std::min(dentCount, n);
    return std::vector<int>(edges.begin(), edges.begin() + dentCount);
}

int findVertexIndex(const std::vector<Point>& poly, const Point& target, double tol) {
    int best = -1;
    double bestD = tol;
    for (int i = 0; i < static_cast<int>(poly.size()); ++i) {
        const double d = geom::dist(poly[static_cast<size_t>(i)], target);
        if (d < bestD) {
            bestD = d;
            best = i;
        }
    }
    return best;
}

double reflexAlphaProxy(const Point& a, const Point& b, const Point& c) {
    const double lab = geom::dist(a, b);
    const double lbc = geom::dist(b, c);
    if (lab < 1e-9 || lbc < 1e-9) return 0.0;
    const double ux = (a.x - b.x) / lab;
    const double uy = (a.y - b.y) / lab;
    const double vx = (c.x - b.x) / lbc;
    const double vy = (c.y - b.y) / lbc;
    const double dot = std::max(-1.0, std::min(1.0, ux * vx + uy * vy));
    return kPi - std::acos(dot);
}

bool insertPocketWithDepths(
    std::vector<Point>& poly,
    const std::vector<Point>& hull,
    int startIdx,
    int spanEdges,
    double tau,
    const Point& directionU,
    const Point& t1,
    const Point& t2,
    const std::vector<double>& innerDepths,
    PocketRecord* record)
{
    if (innerDepths.empty()) return false;

    const int hn = static_cast<int>(hull.size());
    spanEdges = std::max(1, std::min(spanEdges, hn - 1));
    const int endIdx = (startIdx + spanEdges) % hn;

    const double tol = 1e-4 * scaleD(hull);
    const int ia = findVertexIndex(poly, hull[static_cast<size_t>(startIdx)], tol);
    const int ib = findVertexIndex(poly, hull[static_cast<size_t>(endIdx)], tol);
    if (ia < 0 || ib < 0) return false;

    const int pn = static_cast<int>(poly.size());
    int iLo = ia;
    int iHi = ib;
    if (iLo > iHi) std::swap(iLo, iHi);
    if ((iHi - iLo) > pn / 2) std::swap(iLo, iHi);

    const Point& a = poly[static_cast<size_t>(iLo)];
    const Point u = directionU;
    if (std::hypot(u.x, u.y) < 1e-12) return false;

    std::vector<Point> chain;
    chain.push_back(t1);
    const int nv = static_cast<int>(innerDepths.size());
    for (int j = 0; j < nv; ++j) {
        const double frac = static_cast<double>(j + 1) / static_cast<double>(nv + 1);
        const Point mid{t1.x + (t2.x - t1.x) * frac, t1.y + (t2.y - t1.y) * frac};
        chain.push_back(Point{mid.x + u.x * innerDepths[static_cast<size_t>(j)],
                              mid.y + u.y * innerDepths[static_cast<size_t>(j)]});
    }
    chain.push_back(t2);

    std::vector<Point> trial = poly;
    if (spanEdges > 1 && iLo + 1 < iHi) {
        trial.erase(trial.begin() + iLo + 1, trial.begin() + iHi);
    }
    const int insertAfter = findVertexIndex(trial, a, tol);
    if (insertAfter < 0) return false;

    trial.insert(trial.begin() + insertAfter + 1, chain.begin(), chain.end());
    if (!geom::isSimple(trial)) return false;

    poly = std::move(trial);
    if (record) {
        record->startIdx = startIdx;
        record->spanEdges = spanEdges;
        record->tau = tau;
        record->bridgeChord = geom::dist(
            hull[static_cast<size_t>(startIdx)], hull[static_cast<size_t>(endIdx)]);
        record->mouthWidth = geom::dist(t1, t2);
    }
    return true;
}

bool placeMouthOnEdge(
    const std::vector<Point>& hull,
    int edgeIdx,
    double mouthW,
    std::mt19937& rng,
    Point& t1,
    Point& t2,
    double& tauOut)
{
    const int n = static_cast<int>(hull.size());
    const Point& a = hull[static_cast<size_t>(edgeIdx)];
    const Point& b = hull[static_cast<size_t>((edgeIdx + 1) % n)];
    const double len = geom::dist(a, b);
    if (len < mouthW + 1e-6) return false;

    std::uniform_real_distribution<double> u(0.08, 0.92);
    const double margin = mouthW / len;
    const double tLo = std::max(0.05, u(rng) * (1.0 - margin));
    const double tHi = std::min(0.95, tLo + margin);
    tauOut = 0.5 * (tLo + tHi);
    t1 = Point{a.x + (b.x - a.x) * tLo, a.y + (b.y - a.y) * tLo};
    t2 = Point{a.x + (b.x - a.x) * tHi, a.y + (b.y - a.y) * tHi};
    return geom::dist(t1, t2) >= mouthW * 0.92;
}

bool addRandomPocket(
    std::vector<Point>& poly,
    const std::vector<Point>& hull,
    int edgeIdx,
    double depthScale,
    std::mt19937& rng,
    PocketRecord* rec)
{
    const double D = scaleD(hull);
    const int n = static_cast<int>(hull.size());
    std::uniform_int_distribution<int> spanDist(1, std::max(1, n / 3));
    std::uniform_real_distribution<double> depthU(0.04, 0.32);
    std::uniform_real_distribution<double> mouthU(0.06, 0.32);
    std::uniform_int_distribution<int> vertDist(1, 3);

    const int span = spanDist(rng);
    const double depth = depthU(rng) * D * depthScale;
    const double mouthW = mouthU(rng) * D;

    Point t1{}, t2{};
    double tau = 0.5;
    if (!placeMouthOnEdge(hull, edgeIdx, mouthW, rng, t1, t2, tau)) return false;

    Point u{};
    if (!samplePocketDirection(hull, edgeIdx, rng, 85.0, u)) return false;

    const int nv = vertDist(rng);
    std::vector<double> depths;
    depths.reserve(static_cast<size_t>(nv));
    std::uniform_real_distribution<double> fracU(0.12, 1.0);
    for (int j = 0; j < nv; ++j) depths.push_back(fracU(rng) * depth);
    return insertPocketWithDepths(poly, hull, edgeIdx, span, tau, u, t1, t2, depths, rec);
}

bool addDepthFixedPocket(
    std::vector<Point>& poly,
    const std::vector<Point>& hull,
    int edgeIdx,
    double depthRel,
    std::mt19937& rng,
    PocketRecord* rec)
{
    const double D = scaleD(hull);
    const double depth = std::max(1e-6, depthRel * D);
    const int n = static_cast<int>(hull.size());
    std::uniform_int_distribution<int> spanDist(1, std::max(1, n / 3));
    std::uniform_real_distribution<double> mouthU(0.06, 0.30);
    std::uniform_int_distribution<int> vertDist(1, 4);

    const int span = spanDist(rng);
    const double mouthW = mouthU(rng) * D;
    Point t1{}, t2{};
    double tau = 0.5;
    if (!placeMouthOnEdge(hull, edgeIdx, mouthW, rng, t1, t2, tau)) return false;

    Point u{};
    if (!samplePocketDirection(hull, edgeIdx, rng, 85.0, u)) return false;

    const int nv = vertDist(rng);
    std::vector<double> depths;
    depths.reserve(static_cast<size_t>(nv));
    std::uniform_real_distribution<double> fracU(0.08, 0.98);
    for (int j = 0; j < nv - 1; ++j) depths.push_back(fracU(rng) * depth);
    depths.push_back(depth);

    return insertPocketWithDepths(poly, hull, edgeIdx, span, tau, u, t1, t2, depths, rec);
}

bool addWidthFixedPocket(
    std::vector<Point>& poly,
    const std::vector<Point>& hull,
    int edgeIdx,
    double pocketWidthRel,
    std::mt19937& rng,
    PocketRecord* rec)
{
    const double D = scaleD(hull);
    const double mouthW = std::max(1e-6, pocketWidthRel * D);
    const int n = static_cast<int>(hull.size());
    std::uniform_int_distribution<int> spanDist(1, std::max(1, n / 3));
    std::uniform_real_distribution<double> depthU(0.05, 0.35);
    std::uniform_int_distribution<int> extraDist(0, 2);

    Point t1{}, t2{};
    double tau = 0.5;
    if (!placeMouthOnEdge(hull, edgeIdx, mouthW, rng, t1, t2, tau)) return false;

    Point u{};
    if (!samplePocketDirection(hull, edgeIdx, rng, 85.0, u)) return false;

    const int span = spanDist(rng);
    const double depth = depthU(rng) * D;
    const int extra = extraDist(rng);
    std::vector<double> depths;
    depths.push_back(depth);
    std::uniform_real_distribution<double> fracU(0.1, 0.85);
    for (int j = 0; j < extra; ++j) depths.push_back(fracU(rng) * depth);

    return insertPocketWithDepths(poly, hull, edgeIdx, span, tau, u, t1, t2, depths, rec);
}

bool addBridgeFixedPocket(
    std::vector<Point>& poly,
    const std::vector<Point>& hull,
    int edgeIdx,
    double bridgeWidthRel,
    std::mt19937& rng,
    PocketRecord* rec)
{
    const int span = chooseBridgeSpan(hull, bridgeWidthRel, rng);
  std::uniform_real_distribution<double> tauDist(0.1, 0.9);
    std::uniform_real_distribution<double> depthU(0.05, 0.35);
    std::uniform_real_distribution<double> mouthU(0.05, 0.28);
    std::uniform_int_distribution<int> vertDist(1, 3);

    const double D = scaleD(hull);
    const double depth = depthU(rng) * D;
    const double mouthW = mouthU(rng) * D;
    const double tau = tauDist(rng);

    const int hn = static_cast<int>(hull.size());
    const int endIdx = (edgeIdx + span) % hn;
    const Point& a = hull[static_cast<size_t>(edgeIdx)];
    const Point& b = hull[static_cast<size_t>(endIdx)];
    const double edgeLen = geom::dist(a, b);
    if (edgeLen < 1e-9) return false;

    const double half = std::min(0.42, 0.5 * mouthW / edgeLen);
    const double tLo = std::max(0.05, tau - half);
    const double tHi = std::min(0.95, tau + half);
    const Point t1{a.x + (b.x - a.x) * tLo, a.y + (b.y - a.y) * tLo};
    const Point t2{a.x + (b.x - a.x) * tHi, a.y + (b.y - a.y) * tHi};

    Point u{};
    if (!samplePocketDirection(hull, edgeIdx, rng, 85.0, u)) return false;

    const int nv = vertDist(rng);
    std::vector<double> depths;
    std::uniform_real_distribution<double> fracU(0.15, 1.0);
    for (int j = 0; j < nv; ++j) depths.push_back(fracU(rng) * depth);

    return insertPocketWithDepths(poly, hull, edgeIdx, span, tau, u, t1, t2, depths, rec);
}

bool addAlphaFixedPocket(
    std::vector<Point>& poly,
    const std::vector<Point>& hull,
    int edgeIdx,
    double alphaTarget,
    std::mt19937& rng,
    PocketRecord* rec)
{
    const double D = scaleD(hull);
    std::uniform_real_distribution<double> mouthU(0.08, 0.25);
    const double mouthW = mouthU(rng) * D;
    Point t1{}, t2{};
    double tau = 0.5;
    if (!placeMouthOnEdge(hull, edgeIdx, mouthW, rng, t1, t2, tau)) return false;

    Point u{};
    if (!samplePocketDirection(hull, edgeIdx, rng, 60.0, u)) return false;

    const Point mid{0.5 * (t1.x + t2.x), 0.5 * (t1.y + t2.y)};
    double lo = 0.02 * D;
    double hi = 0.45 * D;
    double bestDepth = lo;
    double bestErr = 1e9;

    for (int iter = 0; iter < 28; ++iter) {
        const double d = 0.5 * (lo + hi);
        const Point r{mid.x + u.x * d, mid.y + u.y * d};
        const double a = reflexAlphaProxy(t1, r, t2);
        const double err = std::abs(a - alphaTarget);
        if (err < bestErr) {
            bestErr = err;
            bestDepth = d;
        }
        if (a < alphaTarget) lo = d;
        else hi = d;
    }

    std::uniform_int_distribution<int> spanDist(1, std::max(1, static_cast<int>(hull.size()) / 4));
    const int span = spanDist(rng);
    return insertPocketWithDepths(poly, hull, edgeIdx, span, tau, u, t1, t2, {bestDepth}, rec);
}

}  // namespace

bool applyFreeRandomPockets(
    std::vector<Point>& poly,
    const std::vector<Point>& hull,
    const std::string& fixedAxis,
    double fixedValue,
    std::mt19937& rng,
    EdgePocketBuildResult* out)
{
    if (hull.size() < 3) return false;

    const int dentCount = randomDentCount(rng);
    const std::vector<int> edges = pickRandomEdgeStarts(static_cast<int>(hull.size()), dentCount, rng);
    if (edges.empty()) return false;

    std::vector<PocketRecord> records;
    records.reserve(edges.size());
    int maxSpan = 1;

    auto convEqualsP0 = [](const std::vector<Point>& p0, const std::vector<Point>& p) {
        const auto ch = geom::convexHull(p);
        const double scale = std::max(geom::diameter(p0), 1e-6);
        const double tol = 1e-3 * scale;
        if (std::abs(geom::polygonArea(p0) - geom::polygonArea(ch)) > tol * scale) return false;
        const int n0 = static_cast<int>(p0.size());
        for (const auto& v : ch) {
            bool onP0 = false;
            for (int i = 0; i < n0; ++i) {
                const Point& a = p0[static_cast<size_t>(i)];
                const Point& b = p0[static_cast<size_t>((i + 1) % n0)];
                if (geom::pointToSegmentDist(v, a, b) <= tol) {
                    onP0 = true;
                    break;
                }
            }
            if (!onP0) return false;
        }
        return true;
    };

    const bool areaFixed = (fixedAxis == "area_ratio");
    double depthScale = 1.0;

    for (int outer = 0; outer < 6; ++outer) {
        poly = hull;
        records.clear();
        maxSpan = 1;
        bool failed = false;

        for (size_t k = 0; k < edges.size(); ++k) {
            const int edgeIdx = edges[k];
            PocketRecord rec;
            bool ok = false;

            if (fixedAxis == "depth_rel") {
                ok = addDepthFixedPocket(poly, hull, edgeIdx, fixedValue, rng, &rec);
            } else if (fixedAxis == "pocket_width_rel") {
                ok = addWidthFixedPocket(poly, hull, edgeIdx, fixedValue, rng, &rec);
            } else if (fixedAxis == "bridge_width_rel") {
                ok = addBridgeFixedPocket(poly, hull, edgeIdx, fixedValue, rng, &rec);
            } else if (fixedAxis == "alpha_lebedev") {
                if (k == 0) {
                    ok = addAlphaFixedPocket(poly, hull, edgeIdx, fixedValue, rng, &rec);
                } else {
                    ok = addRandomPocket(poly, hull, edgeIdx, depthScale, rng, &rec);
                }
            } else if (areaFixed) {
                ok = addRandomPocket(poly, hull, edgeIdx, depthScale, rng, &rec);
            } else {
                ok = addRandomPocket(poly, hull, edgeIdx, depthScale, rng, &rec);
            }

            if (!ok) {
                failed = true;
                break;
            }
            maxSpan = std::max(maxSpan, rec.spanEdges);
            records.push_back(rec);
        }

        if (failed || !geom::isSimple(poly) || !convEqualsP0(hull, poly) || geom::isConvex(poly)) {
            continue;
        }

        if (areaFixed) {
            const PolygonMetrics m = computeMetrics(hull, poly, records);
            const double err = m.area_ratio - fixedValue;
            if (std::abs(err) < 0.04) {
                if (out) {
                    out->ok = true;
                    out->maxSpanEdges = maxSpan;
                    out->pockets = std::move(records);
                }
                return true;
            }
            depthScale *= (err > 0) ? 0.88 : 1.12;
            continue;
        }

        if (out) {
            out->ok = true;
            out->maxSpanEdges = maxSpan;
            out->pockets = std::move(records);
        }
        return true;
    }

    return false;
}
