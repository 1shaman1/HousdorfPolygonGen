#include "case_generator.h"

#include "dent_builder.h"

#include "alpha_lebedev.h"
#include "edge_pocket_builder.h"
#include "random_pocket_builder.h"

#include "geometry.h"

#include "polygon_limits.h"
#include "random_polygon_generator.h"

#include "square_sampler.h"

#include <algorithm>

#include <cmath>

#include <utility>



namespace {



bool convEqualsP0(const std::vector<Point>& p0, const std::vector<Point>& p) {
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
}



bool findFixedSweepAxis(const ExperimentConfig& cfg, std::string& axisOut, double& valueOut) {
    for (const auto& kv : cfg.sweeps) {
        if (kv.second.min == kv.second.max) {
            axisOut = kv.first;
            valueOut = kv.second.min;
            return true;
        }
    }
    return false;
}

bool usesFreeRandomGeometry(const std::string& sweepMode) {
    return sweepMode == "fixed_free" || sweepMode == "one_at_a_time" || sweepMode == "target_bins";
}

bool resolveControlledMetric(
    const ExperimentConfig& cfg,
    const GenJob& job,
    std::string& axisOut,
    double& valueOut)
{
    if (cfg.sweep_mode == "fixed_free") {
        return findFixedSweepAxis(cfg, axisOut, valueOut);
    }
    if (cfg.sweep_mode == "one_at_a_time" || cfg.sweep_mode == "target_bins") {
        if (job.sweep_axis.empty() || job.sweep_axis == "none" || job.sweep_axis == "full" ||
            job.sweep_axis == "lhs" || job.sweep_axis == "fixed_free" || job.sweep_axis == "random") {
            return false;
        }
        axisOut = job.sweep_axis;
        valueOut = job.sweep_level;
        return true;
    }
    return false;
}

}  // namespace



GeneratedCase generatePocketCase(const ExperimentConfig& cfg, const GenJob& job) {

    GeneratedCase result;

    const int maxAttempts =
        (cfg.sweep_mode == "target_bins") ? 3 : std::max(1, cfg.max_attempts);

    const bool freeGeom = usesFreeRandomGeometry(cfg.sweep_mode);
    std::string fixedAxis;
    double fixedValue = 0.0;
    if (freeGeom && !resolveControlledMetric(cfg, job, fixedAxis, fixedValue)) {
        return result;
    }



    for (int attempt = 0; attempt < maxAttempts; ++attempt) {

        std::mt19937 attemptRng(job.seed + static_cast<unsigned>(attempt * 9973u));

        auto cloud = sampleSquare(cfg.square, cfg.points_per_hull, attemptRng);

        auto p0 = geom::convexHull(cloud);

        if (static_cast<int>(p0.size()) < cfg.min_hull_vertices) continue;



        std::vector<Point> p = p0;

        EdgePocketBuildResult edgeInfo;

        if (freeGeom) {
            if (!applyFreeRandomPockets(p, p0, fixedAxis, fixedValue, attemptRng, &edgeInfo)) continue;
        } else {
            DentTargets dt;
            dt.depth_rel = job.targets.depth_rel;
            dt.bridge_width_rel = job.targets.bridge_width_rel;
            dt.pocket_width_rel = job.targets.pocket_width_rel;
            dt.alpha_lebedev = job.targets.alpha_lebedev;
            if (!applyEdgePockets(p, p0, dt, cfg.dent_count, attemptRng, &edgeInfo)) continue;
        }

        if (!geom::isSimple(p)) continue;

        if (!convEqualsP0(p0, p)) continue;

        if (!pairWithinVertexLimit(p0, p)) continue;



        PolygonMetrics m = computeMetrics(p0, p, edgeInfo.pockets);
        m.alpha_lebedev = computeAlphaLebedev(p);

        result.p0 = std::move(p0);

        result.p = std::move(p);

        result.metrics = m;

        result.chainK = edgeInfo.maxSpanEdges;

        result.ok = true;

        return result;

    }

    return result;

}

GeneratedCase generateCase(const ExperimentConfig& cfg, const GenJob& job) {
    if (cfg.gen_mode == "random_hull") {
        return generateRandomHullCase(cfg, job);
    }
    if (cfg.gen_mode == "convex_blobs") {
        return generateConvexBlobsCase(cfg, job);
    }
    return generatePocketCase(cfg, job);
}


