#include "case_generator.h"

#include "dent_builder.h"

#include "alpha_lebedev.h"
#include "edge_pocket_builder.h"

#include "geometry.h"

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



}  // namespace



GeneratedCase generateCase(const ExperimentConfig& cfg, const GenJob& job) {

    GeneratedCase result;

    const int maxAttempts =
        (cfg.sweep_mode == "target_bins") ? 3 : std::max(1, cfg.max_attempts);



    for (int attempt = 0; attempt < maxAttempts; ++attempt) {

        std::mt19937 attemptRng(job.seed + static_cast<unsigned>(attempt * 9973u));

        auto cloud = sampleSquare(cfg.square, cfg.points_per_hull, attemptRng);

        auto p0 = geom::convexHull(cloud);

        if (static_cast<int>(p0.size()) < cfg.min_hull_vertices) continue;



        std::vector<Point> p = p0;

        DentTargets dt;

        dt.depth_rel = job.targets.depth_rel;

        dt.bridge_width_rel = job.targets.bridge_width_rel;

        dt.pocket_width_rel = job.targets.pocket_width_rel;

        dt.area_ratio = job.targets.area_ratio;

        dt.alpha_lebedev = job.targets.alpha_lebedev;



        EdgePocketBuildResult edgeInfo;

        if (!applyEdgePockets(p, p0, dt, cfg.dent_count, attemptRng, &edgeInfo)) continue;

        if (!geom::isSimple(p)) continue;

        if (!convEqualsP0(p0, p)) continue;



        PolygonMetrics m = computeMetrics(p0, p, edgeInfo.pockets);
        m.alpha_lebedev = computeAlphaLebedev(p);

        if (cfg.sweep_mode == "target_bins") {

            const double alphaErr = std::abs(m.alpha_proxy - job.targets.alpha_lebedev);

            if (alphaErr > 0.15 && attempt + 1 < maxAttempts) continue;

        }



        result.p0 = std::move(p0);

        result.p = std::move(p);

        result.metrics = m;

        result.chainK = edgeInfo.maxSpanEdges;

        result.ok = true;

        return result;

    }

    return result;

}


