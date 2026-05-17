#pragma once

#include "Point.h"  // before any Windows GDI headers
#include <vector>

struct PocketRecord {
    int startIdx = 0;
    int spanEdges = 1;
    double tau = 0.5;
    double bridgeChord = 0.0;
    double mouthWidth = 0.0;
};

struct PolygonMetrics {
    double area_ratio = 1.0;
    double depth_rel = 0.0;
    double bridge_width_rel = 0.0;
    double pocket_width_rel = 0.0;
    double alpha_proxy = 0.0;
    double alpha_lebedev = 0.0;
    int reflex_count = 0;
    int n_hull = 0;
};

PolygonMetrics computeMetrics(
    const std::vector<Point>& p0,
    const std::vector<Point>& p,
    int pocketChainK = 1);

PolygonMetrics computeMetrics(
    const std::vector<Point>& p0,
    const std::vector<Point>& p,
    const std::vector<PocketRecord>& pockets);
