#include "random_polygon_generator.h"

#include "alpha_lebedev.h"
#include "experiment_config.h"
#include "geometry.h"
#include "metric_calculator.h"
#include "polygon_limits.h"
#include "square_sampler.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <random>
#include <unordered_map>
#include <vector>

namespace {

constexpr int kMinBlobVertices = 4;
constexpr int kMaxBlobVertices = 8;
constexpr int kMinBlobCount = 3;
constexpr int kMaxBlobCount = 8;
constexpr int kMaxGapBridges = 48;

double signedArea(const std::vector<Point>& poly) {
    if (poly.size() < 3) return 0.0;
    double a = 0.0;
    for (size_t i = 0; i < poly.size(); ++i) {
        const size_t j = (i + 1) % poly.size();
        a += poly[i].x * poly[j].y - poly[j].x * poly[i].y;
    }
    return a * 0.5;
}

void ensureCounterClockwise(std::vector<Point>& poly) {
    if (signedArea(poly) < 0) {
        std::reverse(poly.begin(), poly.end());
    }
}

double grahamCross(const Point& o, const Point& a, const Point& b) {
    return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
}

std::vector<Point> grahamConvexHull(std::vector<Point> pts) {
    if (pts.size() <= 1) return pts;

    const auto pivotIt = std::min_element(
        pts.begin(), pts.end(),
        [](const Point& a, const Point& b) {
            if (a.y != b.y) return a.y < b.y;
            return a.x < b.x;
        });
    std::swap(pts.front(), *pivotIt);
    const Point pivot = pts.front();

    std::sort(pts.begin() + 1, pts.end(),
        [&pivot](const Point& a, const Point& b) {
            const double cr = grahamCross(pivot, a, b);
            if (cr != 0.0) return cr > 0.0;
            const double da = (a.x - pivot.x) * (a.x - pivot.x) + (a.y - pivot.y) * (a.y - pivot.y);
            const double db = (b.x - pivot.x) * (b.x - pivot.x) + (b.y - pivot.y) * (b.y - pivot.y);
            return da < db;
        });

    std::vector<Point> hull;
    hull.reserve(pts.size());
    for (const Point& p : pts) {
        while (hull.size() >= 2 &&
               grahamCross(hull[hull.size() - 2], hull.back(), p) <= 0.0) {
            hull.pop_back();
        }
        hull.push_back(p);
    }
    return hull;
}

constexpr int kMinScatterPointsPerBlob = 10;
constexpr int kScatterOversampleFactor = 3;
constexpr int kMaxBlobHullAttempts = 24;

std::vector<Point> sampleConvexBlobFromScatter(
    const Point& center,
    double radius,
    int hullVertexHint,
    std::mt19937& rng)
{
    std::uniform_real_distribution<double> u01(0.0, 1.0);
    const int scatterCount = std::max(
        kMinScatterPointsPerBlob,
        hullVertexHint * kScatterOversampleFactor);

    for (int attempt = 0; attempt < kMaxBlobHullAttempts; ++attempt) {
        std::vector<Point> cloud;
        cloud.reserve(static_cast<size_t>(scatterCount));
        for (int i = 0; i < scatterCount; ++i) {
            const double theta = u01(rng) * 2.0 * 3.14159265358979323846;
            const double r = radius * std::sqrt(u01(rng));
            cloud.push_back(
                {center.x + r * std::cos(theta), center.y + r * std::sin(theta)});
        }

        std::vector<Point> hull = grahamConvexHull(std::move(cloud));
        if (hull.size() < static_cast<size_t>(kMinBlobVertices)) {
            continue;
        }
        ensureCounterClockwise(hull);
        return hull;
    }
    return {};
}

bool isExteriorUnionEdge(
    const Point& a,
    const Point& b,
    const std::vector<std::vector<Point>>& blobs,
    double eps)
{
    const double mx = (a.x + b.x) * 0.5;
    const double my = (a.y + b.y) * 0.5;
    const double ex = b.x - a.x;
    const double ey = b.y - a.y;
    const double len = std::hypot(ex, ey);
    if (len < 1e-12) return false;

    // For CCW boundary a->b, exterior lies to the right of the directed edge.
    const double nx = ey / len;
    const double ny = -ex / len;
    const Point probe{mx + nx * eps, my + ny * eps};

    for (const auto& poly : blobs) {
        if (geom::pointInConvexPolygon(poly, probe, true)) {
            return false;
        }
    }
    return true;
}

struct QuantKey {
    int64_t x = 0;
    int64_t y = 0;

    bool operator==(const QuantKey& o) const { return x == o.x && y == o.y; }
    bool operator!=(const QuantKey& o) const { return !(*this == o); }
};

struct QuantHash {
    size_t operator()(const QuantKey& k) const noexcept {
        return static_cast<size_t>(k.x * 73856093LL ^ k.y * 19349663LL);
    }
};

QuantKey quantize(const Point& p, double invEps) {
    return {static_cast<int64_t>(std::llround(p.x * invEps)),
            static_cast<int64_t>(std::llround(p.y * invEps))};
}

QuantKey lowestKey(const std::unordered_map<QuantKey, Point, QuantHash>& keyToPoint) {
    QuantKey start = keyToPoint.begin()->first;
    for (const auto& kv : keyToPoint) {
        const Point& p = kv.second;
        const Point& ps = keyToPoint.at(start);
        if (p.y < ps.y - 1e-12 || (std::abs(p.y - ps.y) <= 1e-12 && p.x < ps.x)) {
            start = kv.first;
        }
    }
    return start;
}

QuantKey findNearestBlobVertexKey(
    const Point& from,
    const QuantKey& exclude,
    const std::vector<std::vector<Point>>& blobs,
    double invEps)
{
    QuantKey best;
    bool found = false;
    double bestDist = 0.0;

    for (const auto& poly : blobs) {
        for (const Point& v : poly) {
            const QuantKey key = quantize(v, invEps);
            if (key == exclude) continue;
            const double d = geom::dist(from, v);
            if (!found || d < bestDist) {
                found = true;
                bestDist = d;
                best = key;
            }
        }
    }
    return found ? best : exclude;
}

bool walkExteriorCycle(
    const std::unordered_map<QuantKey, QuantKey, QuantHash>& nextKey,
    const std::unordered_map<QuantKey, Point, QuantHash>& keyToPoint,
    QuantKey start,
    std::vector<Point>& walk,
    std::unordered_map<QuantKey, bool, QuantHash>& visited)
{
    walk.clear();
    visited.clear();
    if (nextKey.empty()) return false;

    QuantKey cur = start;
    const size_t guardLimit = nextKey.size() + 4;
    for (size_t guard = 0; guard < guardLimit; ++guard) {
        walk.push_back(keyToPoint.at(cur));
        visited[cur] = true;

        const auto it = nextKey.find(cur);
        if (it == nextKey.end()) return false;

        cur = it->second;
        if (cur == start) {
            return walk.size() >= 4;
        }
    }
    return false;
}

bool stitchExteriorBoundaryWithGapBridges(
    const std::vector<std::pair<Point, Point>>& directedEdges,
    const std::vector<std::vector<Point>>& blobs,
    double invEps,
    std::vector<Point>& out)
{
    std::unordered_map<QuantKey, Point, QuantHash> keyToPoint;
    std::unordered_map<QuantKey, QuantKey, QuantHash> nextKey;

    for (const auto& e : directedEdges) {
        const QuantKey from = quantize(e.first, invEps);
        const QuantKey to = quantize(e.second, invEps);
        keyToPoint[from] = e.first;
        keyToPoint[to] = e.second;
        nextKey[from] = to;
    }

    if (nextKey.empty()) return false;

    const QuantKey start = lowestKey(keyToPoint);

    for (int bridge = 0; bridge < kMaxGapBridges; ++bridge) {
        std::vector<Point> walk;
        std::unordered_map<QuantKey, bool, QuantHash> visited;
        if (walkExteriorCycle(nextKey, keyToPoint, start, walk, visited)) {
            if (visited.size() == nextKey.size()) {
                out = std::move(walk);
                return true;
            }
        } else if (!walk.empty()) {
            const QuantKey openEnd = quantize(walk.back(), invEps);
            const Point& fromPt = keyToPoint.at(openEnd);
            const QuantKey nearest =
                findNearestBlobVertexKey(fromPt, openEnd, blobs, invEps);
            if (nearest == openEnd) return false;
            for (const auto& poly : blobs) {
                for (const Point& v : poly) {
                    if (quantize(v, invEps) == nearest) {
                        keyToPoint[nearest] = v;
                        break;
                    }
                }
            }
            nextKey[openEnd] = nearest;
            continue;
        }

        QuantKey bridgeFrom = start;
        for (const auto& kv : nextKey) {
            if (!visited.count(kv.first)) {
                bridgeFrom = kv.first;
                break;
            }
        }

        const Point& fromPt = keyToPoint.at(bridgeFrom);
        QuantKey nearest = findNearestBlobVertexKey(fromPt, bridgeFrom, blobs, invEps);
        if (nearest == bridgeFrom) return false;

        if (!visited.count(bridgeFrom) && !visited.empty()) {
            QuantKey bestVisited = start;
            double bestDist = geom::dist(fromPt, keyToPoint.at(bestVisited));
            for (const auto& kv : visited) {
                const double d = geom::dist(fromPt, keyToPoint.at(kv.first));
                if (d < bestDist) {
                    bestDist = d;
                    bestVisited = kv.first;
                }
            }
            bridgeFrom = bestVisited;
            nearest = findNearestBlobVertexKey(
                keyToPoint.at(bridgeFrom), bridgeFrom, blobs, invEps);
            if (nearest == bridgeFrom) return false;
        }

        for (const auto& poly : blobs) {
            for (const Point& v : poly) {
                if (quantize(v, invEps) == nearest) {
                    keyToPoint[nearest] = v;
                    break;
                }
            }
        }
        nextKey[bridgeFrom] = nearest;
    }

    return false;
}

bool mergeBlobsWithClosestBridges(
    const std::vector<std::vector<Point>>& blobs,
    std::vector<Point>& out)
{
    if (blobs.empty()) return false;
    out = blobs.front();
    if (out.size() < 3) return false;
    ensureCounterClockwise(out);

    for (size_t k = 1; k < blobs.size(); ++k) {
        const std::vector<Point>& b = blobs[k];
        if (b.size() < 3) return false;

        size_t bestI = 0;
        size_t bestJ = 0;
        double bestD = 1e300;
        for (size_t i = 0; i < out.size(); ++i) {
            for (size_t j = 0; j < b.size(); ++j) {
                const double d = geom::dist(out[i], b[j]);
                if (d < bestD) {
                    bestD = d;
                    bestI = i;
                    bestJ = j;
                }
            }
        }

        std::vector<Point> merged;
        merged.reserve(out.size() + b.size() + 2);
        for (size_t t = 0; t <= bestI; ++t) {
            merged.push_back(out[t]);
        }
        for (size_t t = 0; t < b.size(); ++t) {
            merged.push_back(b[(bestJ + t) % b.size()]);
        }
        for (size_t t = bestI + 1; t < out.size(); ++t) {
            merged.push_back(out[t]);
        }
        out = std::move(merged);
        ensureCounterClockwise(out);
    }
    return out.size() >= 4;
}

bool buildUnionOuterBoundary(
    const std::vector<std::vector<Point>>& blobs,
    double geomScale,
    std::vector<Point>& out)
{
    std::vector<std::vector<Point>> ccw = blobs;
    for (auto& poly : ccw) {
        ensureCounterClockwise(poly);
    }

    const double eps = std::max(geomScale * 1e-9, 1e-9);
    const double invEps = 1.0 / eps;

    std::vector<std::pair<Point, Point>> exterior;
    for (const auto& poly : ccw) {
        const int n = static_cast<int>(poly.size());
        for (int i = 0; i < n; ++i) {
            const Point& a = poly[static_cast<size_t>(i)];
            const Point& b = poly[static_cast<size_t>((i + 1) % n)];
            if (isExteriorUnionEdge(a, b, ccw, eps)) {
                exterior.emplace_back(a, b);
            }
        }
    }

    if (exterior.size() < 4) return false;
    return stitchExteriorBoundaryWithGapBridges(exterior, ccw, invEps, out);
}

SquareBounds placeBlobWindowRandomly(const ExperimentConfig& cfg, std::mt19937& rng) {
    const SquareBounds& global = cfg.square;
    const double globW = global.xmax - global.xmin;
    const double globH = global.ymax - global.ymin;

    double winW = globW;
    double winH = globH;
    if (cfg.has_blob_square) {
        winW = cfg.blob_square.xmax - cfg.blob_square.xmin;
        winH = cfg.blob_square.ymax - cfg.blob_square.ymin;
        winW = std::min(winW, globW);
        winH = std::min(winH, globH);
    }

    if (winW <= 0.0 || winH <= 0.0) {
        return global;
    }

    std::uniform_real_distribution<double> u01(0.0, 1.0);
    const double slackX = globW - winW;
    const double slackY = globH - winH;
    const double x0 = global.xmin + (slackX > 0.0 ? u01(rng) * slackX : 0.0);
    const double y0 = global.ymin + (slackY > 0.0 ? u01(rng) * slackY : 0.0);
    return {x0, x0 + winW, y0, y0 + winH};
}

bool tryGenerateConvexBlobs(
    const ExperimentConfig& cfg,
    std::mt19937& rng,
    std::vector<Point>& nonConvexOut)
{
    const double globalScale = std::max(
        cfg.square.xmax - cfg.square.xmin,
        cfg.square.ymax - cfg.square.ymin);

    std::uniform_int_distribution<int> blobCountDist(kMinBlobCount, kMaxBlobCount);
    std::uniform_int_distribution<int> vertexCountDist(kMinBlobVertices, kMaxBlobVertices);
    std::uniform_real_distribution<double> u01(0.0, 1.0);

    const int blobCount = blobCountDist(rng);

    std::vector<std::vector<Point>> blobs;
    blobs.reserve(static_cast<size_t>(blobCount));
    for (int i = 0; i < blobCount; ++i) {
        const SquareBounds window = placeBlobWindowRandomly(cfg, rng);
        const double width = window.xmax - window.xmin;
        const double height = window.ymax - window.ymin;
        const double winScale = std::max(width, height);
        const double margin = 0.12 * winScale;
        const double baseRadius = (0.10 + 0.06 * u01(rng)) * winScale;

        Point center{};
        bool placed = false;
        for (int attempt = 0; attempt < 48; ++attempt) {
            if (width <= 2.0 * margin || height <= 2.0 * margin) {
                break;
            }
            center = {
                window.xmin + margin + u01(rng) * (width - 2.0 * margin),
                window.ymin + margin + u01(rng) * (height - 2.0 * margin)};
            placed = true;
            break;
        }
        if (!placed) {
            center = {
                (window.xmin + window.xmax) * 0.5,
                (window.ymin + window.ymax) * 0.5};
        }

        const int vtx = vertexCountDist(rng);
        const double radius = baseRadius * (0.75 + 0.5 * u01(rng));
        std::vector<Point> blob =
            sampleConvexBlobFromScatter(center, radius, vtx, rng);
        if (blob.size() < static_cast<size_t>(kMinBlobVertices)) {
            return false;
        }
        blobs.push_back(std::move(blob));
    }

    std::vector<Point> boundary;
    if (!buildUnionOuterBoundary(blobs, globalScale, boundary) &&
        !mergeBlobsWithClosestBridges(blobs, boundary)) {
        return false;
    }
    if (!polygonWithinVertexLimit(boundary)) return false;

    geom::untanglePolygon2Opt(boundary);
    if (!geom::isSimple(boundary)) return false;
    if (geom::isConvex(boundary)) return false;

    nonConvexOut = std::move(boundary);
    return true;
}

GeneratedCase finishRandomNonConvexCase(
    const ExperimentConfig& cfg,
    std::vector<Point> p)
{
    GeneratedCase result;
    if (!geom::isSimple(p)) return result;
    if (geom::isConvex(p)) return result;

    std::vector<Point> p0 = geom::convexHull(p);
    if (static_cast<int>(p0.size()) < cfg.min_hull_vertices) return result;
    if (!pairWithinVertexLimit(p0, p)) return result;

    PolygonMetrics m = computeMetrics(p0, p, {});
    m.alpha_lebedev = computeAlphaLebedev(p);

    result.p0 = std::move(p0);
    result.p = std::move(p);
    result.metrics = m;
    result.chainK = 0;
    result.ok = true;
    return result;
}

}  // namespace

GeneratedCase generateRandomHullCase(const ExperimentConfig& cfg, const GenJob& job) {
    GeneratedCase result;
    const int maxAttempts = std::max(1, cfg.max_attempts);

    for (int attempt = 0; attempt < maxAttempts; ++attempt) {
        std::mt19937 rng(job.seed + static_cast<unsigned>(attempt * 7919u));

        std::uniform_int_distribution<int> countDist(
            kMinRandomPolygonVertices, kMaxPolygonVertices);
        const int vertexCount = countDist(rng);

        std::vector<Point> p = sampleSquare(cfg.square, vertexCount, rng);
        geom::untanglePolygon2Opt(p);

        result = finishRandomNonConvexCase(cfg, std::move(p));
        if (result.ok) return result;
    }

    return result;
}

GeneratedCase generateConvexBlobsCase(const ExperimentConfig& cfg, const GenJob& job) {
    GeneratedCase result;
    const int maxAttempts = std::max(1, cfg.max_attempts);

    for (int attempt = 0; attempt < maxAttempts; ++attempt) {
        std::mt19937 rng(job.seed + static_cast<unsigned>(attempt * 7919u));

        std::vector<Point> p;
        if (!tryGenerateConvexBlobs(cfg, rng, p)) continue;

        result = finishRandomNonConvexCase(cfg, std::move(p));
        if (result.ok) return result;
    }

    return result;
}
