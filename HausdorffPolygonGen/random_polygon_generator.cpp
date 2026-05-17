#include "random_polygon_generator.h"

#include "alpha_lebedev.h"
#include "geometry.h"
#include "metric_calculator.h"
#include "polygon_limits.h"
#include "square_sampler.h"

#include <algorithm>
#include <random>

namespace {

void removeIntersections(std::vector<Point>& p) {
    const int n = static_cast<int>(p.size());
    if (n < 4) return;

    bool improved = true;
    while (improved) {
        improved = false;
        for (int i = 0; i < n; ++i) {
            const int i2 = (i + 1) % n;
            for (int j = i + 2; j < n; ++j) {
                const int j2 = (j + 1) % n;
                if (i == j2) continue;
                if (geom::segmentsIntersect(
                        p[static_cast<size_t>(i)],
                        p[static_cast<size_t>(i2)],
                        p[static_cast<size_t>(j)],
                        p[static_cast<size_t>(j2)])) {
                    std::reverse(p.begin() + i + 1, p.begin() + j + 1);
                    improved = true;
                }
            }
        }
    }
}

std::vector<Point> sampleRandomPolygon(
    const SquareBounds& square,
    int vertexCount,
    std::mt19937& rng)
{
    std::uniform_real_distribution<double> ux(square.xmin, square.xmax);
    std::uniform_real_distribution<double> uy(square.ymin, square.ymax);

    std::vector<Point> points;
    points.reserve(static_cast<size_t>(vertexCount));
    for (int i = 0; i < vertexCount; ++i) {
        points.push_back({ux(rng), uy(rng)});
    }
    removeIntersections(points);
    return points;
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

        std::vector<Point> p = sampleRandomPolygon(cfg.square, vertexCount, rng);
        if (!geom::isSimple(p)) continue;
        if (geom::isConvex(p)) continue;

        std::vector<Point> p0 = geom::convexHull(p);
        if (static_cast<int>(p0.size()) < cfg.min_hull_vertices) continue;
        if (!pairWithinVertexLimit(p0, p)) continue;

        PolygonMetrics m = computeMetrics(p0, p, {});
        m.alpha_lebedev = computeAlphaLebedev(p);

        result.p0 = std::move(p0);
        result.p = std::move(p);
        result.metrics = m;
        result.chainK = 0;
        result.ok = true;
        return result;
    }

    return result;
}
