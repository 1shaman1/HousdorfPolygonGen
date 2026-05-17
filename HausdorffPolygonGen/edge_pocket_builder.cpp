#include "edge_pocket_builder.h"

#include "geometry.h"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace {

constexpr double kPi = 3.14159265358979323846;

double scaleD(const std::vector<Point>& hull) {
    const double a = geom::polygonArea(hull);
    if (a > 1e-12) return std::sqrt(a);
    return std::max(geom::diameter(hull), 1e-6);
}

Point normalize(const Point& v) {
    const double l = geom::dist({0, 0}, v);
    if (l < 1e-12) return {0, 0};
    return {v.x / l, v.y / l};
}

Point rotate(const Point& v, double rad) {
    const double c = std::cos(rad);
    const double s = std::sin(rad);
    return {v.x * c - v.y * s, v.x * s + v.y * c};
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

Point innerNormal(const std::vector<Point>& hull, int edgeIdx) {
    const int n = static_cast<int>(hull.size());
    const Point& a = hull[static_cast<size_t>(edgeIdx)];
    const Point& b = hull[static_cast<size_t>((edgeIdx + 1) % n)];
    Point e{b.x - a.x, b.y - a.y};
    Point nLeft{-e.y, e.x};
    Point nRight{e.y, -e.x};
    const double cx = (a.x + b.x) * 0.5;
    const double cy = (a.y + b.y) * 0.5;
    double sx = 0, sy = 0;
    for (const auto& p : hull) {
        sx += p.x;
        sy += p.y;
    }
    const double inv = 1.0 / std::max(1, n);
    const Point g{sx * inv, sy * inv};
    const Point toG{g.x - cx, g.y - cy};
    const double dL = nLeft.x * toG.x + nLeft.y * toG.y;
    return normalize(dL > 0 ? nLeft : nRight);
}

bool pointInsideConvex(const std::vector<Point>& hull, const Point& p, double eps) {
    const int n = static_cast<int>(hull.size());
    for (int i = 0; i < n; ++i) {
        const Point& a = hull[static_cast<size_t>(i)];
        const Point& b = hull[static_cast<size_t>((i + 1) % n)];
        if (geom::cross(a, b, p) < -eps) return false;
    }
    return true;
}

bool directionInWedge(const std::vector<Point>& hull, int edgeIdx, const Point& u) {
    const int n = static_cast<int>(hull.size());
    const int prev = (edgeIdx - 1 + n) % n;
    const Point& a = hull[static_cast<size_t>(edgeIdx)];
    const Point& b = hull[static_cast<size_t>((edgeIdx + 1) % n)];
    const Point& cPrev = hull[static_cast<size_t>(prev)];
    const Point& cNext = hull[static_cast<size_t>((edgeIdx + 2) % n)];

    const Point ePrev{a.x - cPrev.x, a.y - cPrev.y};
    const Point eNext{cNext.x - b.x, cNext.y - b.y};
    const Point nIn = innerNormal(hull, edgeIdx);

    const auto okHalf = [&](const Point& edgeOut, const Point& outward) {
        const double cr = edgeOut.x * u.y - edgeOut.y * u.x;
        const double co = outward.x * u.x + outward.y * u.y;
        return cr * co >= -1e-9;
    };

    return okHalf(ePrev, {-ePrev.y, ePrev.x}) && okHalf(eNext, {eNext.y, -eNext.x}) &&
           (nIn.x * u.x + nIn.y * u.y) > 0.05;
}

std::vector<int> pickPocketStarts(int n, int dentCount, int minGap) {
    std::vector<int> starts;
    if (n < 3 || dentCount <= 0) return starts;
    dentCount = std::min(dentCount, std::max(1, n / std::max(2, minGap + 1)));
    const int step = std::max(minGap + 1, n / dentCount);
    for (int k = 0; k < dentCount; ++k) starts.push_back((k * step) % n);
    return starts;
}

double bridgeChordForSpan(const std::vector<Point>& hull, int start, int span) {
    const int n = static_cast<int>(hull.size());
    return geom::dist(hull[static_cast<size_t>(start)], hull[static_cast<size_t>((start + span) % n)]);
}

int pocketVertexCount(double pocketWidthRel, std::mt19937& rng) {
    std::uniform_int_distribution<int> d(1, 3);
    int m = d(rng);
    if (pocketWidthRel > 0.22) m = std::max(m, 2);
    if (pocketWidthRel > 0.32) m = 3;
    return m;
}

bool insertChainBetween(
    std::vector<Point>& poly,
    int idxAfter,
    const std::vector<Point>& chain)
{
    if (chain.empty()) return true;
    poly.insert(poly.begin() + idxAfter + 1, chain.begin(), chain.end());
    return geom::isSimple(poly);
}

}  // namespace

bool samplePocketDirection(
    const std::vector<Point>& hull,
    int edgeIdx,
    std::mt19937& rng,
    double thetaMaxDeg,
    Point& outUnit)
{
    const Point nIn = innerNormal(hull, edgeIdx);
    if (nIn.x == 0 && nIn.y == 0) return false;

    std::uniform_real_distribution<double> thetaDist(
        -thetaMaxDeg * kPi / 180.0, thetaMaxDeg * kPi / 180.0);
    for (int attempt = 0; attempt < 24; ++attempt) {
        const Point u = normalize(rotate(nIn, thetaDist(rng)));
        if (directionInWedge(hull, edgeIdx, u)) {
            outUnit = u;
            return true;
        }
    }
    outUnit = nIn;
    return true;
}

int chooseBridgeSpan(
    const std::vector<Point>& hull,
    double bridgeWidthRelTarget,
    std::mt19937& rng)
{
    const int n = static_cast<int>(hull.size());
    if (n < 3) return 1;
    const double L = geom::perimeter(hull);
    const double target = std::max(1e-9, bridgeWidthRelTarget * L);

    std::vector<std::pair<int, double>> candidates;
    for (int start = 0; start < n; ++start) {
        for (int k = 1; k < n / 2; ++k) {
            const double chord = bridgeChordForSpan(hull, start, k);
            candidates.emplace_back(k, std::abs(chord - target));
        }
    }
    std::sort(candidates.begin(), candidates.end(), [](const auto& a, const auto& b) {
        return a.second < b.second;
    });
    if (candidates.empty()) return 1;

    const int top = std::min(5, static_cast<int>(candidates.size()));
    std::uniform_int_distribution<int> pick(0, top - 1);
    return std::max(1, candidates[static_cast<size_t>(pick(rng))].first);
}

bool insertPocketOnEdge(
    std::vector<Point>& poly,
    const std::vector<Point>& hull,
    int startIdx,
    int spanEdges,
    double tau,
    const Point& directionU,
    double depth,
    int pocketVerts,
    double mouthWidthTarget,
    PocketRecord* record)
{
    const int hn = static_cast<int>(hull.size());
    if (hn < 3 || spanEdges < 1) return false;

    spanEdges = std::min(spanEdges, hn - 1);
    const int endIdx = (startIdx + spanEdges) % hn;

    const double tol = 1e-4 * scaleD(hull);
    const int ia = findVertexIndex(poly, hull[static_cast<size_t>(startIdx)], tol);
    const int ib = findVertexIndex(poly, hull[static_cast<size_t>(endIdx)], tol);
    if (ia < 0 || ib < 0) return false;

    const int pn = static_cast<int>(poly.size());
    int iLo = ia;
    int iHi = ib;
    if (iLo > iHi) std::swap(iLo, iHi);

    const bool wraps = (iHi - iLo) > pn / 2;
    if (wraps) {
        std::swap(iLo, iHi);
    }

    const Point& a = poly[static_cast<size_t>(iLo)];
    const Point& b = poly[static_cast<size_t>(iHi)];

    const Point u = normalize(directionU);
    if (u.x == 0 && u.y == 0) return false;

    const double D = scaleD(hull);
    const double edgeLen = geom::dist(a, b);
    if (edgeLen < 1e-9) return false;

    tau = std::max(0.05, std::min(0.95, tau));
    const double mouthW = (mouthWidthTarget > 1e-9) ? mouthWidthTarget : 0.12 * D;
    const double mouthHalf = std::max(0.02, std::min(0.42, 0.5 * mouthW / edgeLen));
    double tLo = std::max(0.05, tau - mouthHalf);
    double tHi = std::min(0.95, tau + mouthHalf);
    if (tHi - tLo < 0.04) {
        tLo = std::max(0.05, tau - 0.02);
        tHi = std::min(0.95, tau + 0.02);
    }

    const Point t1{a.x + (b.x - a.x) * tLo, a.y + (b.y - a.y) * tLo};
    const Point t2{a.x + (b.x - a.x) * tHi, a.y + (b.y - a.y) * tHi};

    std::vector<Point> chain;
    chain.push_back(t1);
    pocketVerts = std::max(1, pocketVerts);
    for (int j = 1; j <= pocketVerts; ++j) {
        const double frac = static_cast<double>(j) / static_cast<double>(pocketVerts + 1);
        const Point mid{t1.x + (t2.x - t1.x) * frac, t1.y + (t2.y - t1.y) * frac};
        chain.push_back(Point{mid.x + u.x * depth * frac, mid.y + u.y * depth * frac});
    }
    chain.push_back(t2);

    std::vector<Point> trial = poly;
    if (spanEdges > 1) {
        if (iLo + 1 < iHi) {
            trial.erase(trial.begin() + iLo + 1, trial.begin() + iHi);
        }
    }

    const int insertAfter = findVertexIndex(trial, a, tol);
    if (insertAfter < 0) return false;

    if (!insertChainBetween(trial, insertAfter, chain)) {
        const double halfDepth = depth * 0.55;
        chain.clear();
        chain.push_back(t1);
        chain.push_back(Point{t1.x + u.x * halfDepth, t1.y + u.y * halfDepth});
        chain.push_back(t2);
        if (!insertChainBetween(trial, insertAfter, chain)) return false;
    }

    for (const auto& q : chain) {
        if (!pointInsideConvex(hull, q, 1e-3 * scaleD(hull))) {
            return false;
        }
    }

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

bool applyEdgePockets(
    std::vector<Point>& poly,
    const std::vector<Point>& hull,
    const DentTargets& targets,
    int dentCount,
    std::mt19937& rng,
    EdgePocketBuildResult* out)
{
    if (hull.size() < 3) return false;

    const double D = scaleD(hull);
    const int minGap = std::max(2, static_cast<int>(hull.size()) / (2 * std::max(1, dentCount) + 1));
    const std::vector<int> starts = pickPocketStarts(static_cast<int>(hull.size()), dentCount, minGap);
    if (starts.empty()) return false;

    std::vector<PocketRecord> records;
    int maxSpan = 1;

    std::uniform_real_distribution<double> tauDist(0.08, 0.92);
    double depthScale = 1.0;

    for (int calib = 0; calib < 2; ++calib) {
        poly = hull;
        records.clear();
        maxSpan = 1;

        for (int edgeIdx : starts) {
            const int span = chooseBridgeSpan(hull, targets.bridge_width_rel, rng);
            maxSpan = std::max(maxSpan, span);

            Point u{};
            if (!samplePocketDirection(hull, edgeIdx, rng, 75.0, u)) continue;

            const double depth = std::max(1e-6, targets.depth_rel * D * depthScale);
            const int verts = pocketVertexCount(targets.pocket_width_rel, rng);
            const double tau = tauDist(rng);

            PocketRecord rec;
            const double mouthW = targets.pocket_width_rel * D;
            if (!insertPocketOnEdge(poly, hull, edgeIdx, span, tau, u, depth, verts, mouthW, &rec)) {
                poly = hull;
                records.clear();
                break;
            }
            records.push_back(rec);
        }

        if (records.size() != starts.size()) continue;

        if (!geom::isSimple(poly)) {
            depthScale *= 0.75;
            continue;
        }

        const PolygonMetrics m = computeMetrics(hull, poly, records);
        const double err = m.area_ratio - targets.area_ratio;
        if (std::abs(err) < 0.03 || calib == 1) break;
        if (m.area_ratio < targets.area_ratio)
            depthScale *= 1.12;
        else
            depthScale *= 0.88;
    }

    if (records.empty() || !geom::isSimple(poly) || geom::isConvex(poly)) return false;

    if (out) {
        out->ok = true;
        out->maxSpanEdges = maxSpan;
        out->pockets = std::move(records);
    }
    return true;
}
