#include "dent_builder.h"

#include "edge_pocket_builder.h"

#include "geometry.h"

#include "metric_calculator.h"

#include <algorithm>

#include <cmath>



namespace {



double depthCoeffFromTarget(double depthRelTarget) {

    return std::max(0.02, std::min(0.85, depthRelTarget * 2.5));

}



}  // namespace



// Legacy vertex-shift pocket (kept for tests / reference).

bool pushVertexReflex(

    std::vector<Point>& poly,

    const std::vector<Point>& hull,

    int idx,

    double depthCoeff,

    ReflexDirection dir)

{

    const int n = static_cast<int>(poly.size());

    if (n < 3) return false;

    const int i0 = (idx - 1 + n) % n;

    const int i1 = idx;

    const int i2 = (idx + 1) % n;

    const Point A = poly[static_cast<size_t>(i0)];

    const Point B = poly[static_cast<size_t>(i1)];

    const Point C = poly[static_cast<size_t>(i2)];



    const double lenBA = geom::dist(A, B);

    const double lenBC = geom::dist(C, B);

    if (lenBA < 1e-9 || lenBC < 1e-9) return false;



    const double shift = std::min(lenBA, lenBC) * depthCoeff;

    Point moveDir{0, 0};



    if (dir == ReflexDirection::Bisector) {

        const Point u{(A.x - B.x) / lenBA, (A.y - B.y) / lenBA};

        const Point w{(C.x - B.x) / lenBC, (C.y - B.y) / lenBC};

        Point bis{u.x + w.x, u.y + w.y};

        const double l = geom::dist({0, 0}, bis);

        if (l < 1e-9) {

            double sx = 0, sy = 0;

            for (const auto& p : hull) {

                sx += p.x;

                sy += p.y;

            }

            const double inv = 1.0 / std::max<size_t>(1, hull.size());

            Point G{sx * inv, sy * inv};

            Point d{G.x - B.x, G.y - B.y};

            const double gl = geom::dist({0, 0}, d);

            if (gl < 1e-9) return false;

            moveDir = {d.x / gl, d.y / gl};

        } else {

            moveDir = {bis.x / l, bis.y / l};

        }

    } else {

        double sx = 0, sy = 0;

        for (const auto& p : hull) {

            sx += p.x;

            sy += p.y;

        }

        const double inv = 1.0 / std::max<size_t>(1, hull.size());

        Point G{sx * inv, sy * inv};

        Point d{G.x - B.x, G.y - B.y};

        const double l = geom::dist({0, 0}, d);

        if (l < 1e-9) return false;

        moveDir = {d.x / l, d.y / l};

    }



    poly[static_cast<size_t>(i1)].x += moveDir.x * shift;

    poly[static_cast<size_t>(i1)].y += moveDir.y * shift;

    return true;

}



int chooseChainLength(const std::vector<Point>& hull, double bridgeWidthRelTarget, std::mt19937& rng) {

    return chooseBridgeSpan(hull, bridgeWidthRelTarget, rng);

}



bool applyPocketAt(

    std::vector<Point>& poly,

    const std::vector<Point>& hull,

    int startIdx,

    int chainK,

    double depthCoeff,

    ReflexDirection dir)

{

    (void)dir;

    const double D = std::max(geom::diameter(hull), std::sqrt(std::max(geom::polygonArea(hull), 1e-6)));

    const double depth = depthCoeff * D;

    Point u{};

    std::mt19937 dummy(0);

    if (!samplePocketDirection(hull, startIdx, dummy, 70.0, u)) return false;

    return insertPocketOnEdge(poly, hull, startIdx, chainK, 0.5, u, depth, std::max(1, chainK), 0.1 * D);

}



bool applyDents(

    std::vector<Point>& poly,

    const std::vector<Point>& hull,

    const DentTargets& targets,

    int dentCount,

    std::mt19937& rng,

    DentBuildResult* out)

{

    EdgePocketBuildResult edgeResult;

    if (!applyEdgePockets(poly, hull, targets, dentCount, rng, &edgeResult)) return false;



    if (out) {

        out->ok = true;

        out->chainK = edgeResult.maxSpanEdges;

        out->depthCoeff = depthCoeffFromTarget(targets.depth_rel);

    }

    return true;

}


