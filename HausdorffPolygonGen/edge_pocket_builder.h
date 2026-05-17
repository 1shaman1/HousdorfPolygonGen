#pragma once

#include "dent_builder.h"
#include "metric_calculator.h"
#include "Point.h"
#include <random>
#include <vector>

struct EdgePocketBuildResult {
    bool ok = false;
    int maxSpanEdges = 1;
    std::vector<PocketRecord> pockets;
};

bool samplePocketDirection(
    const std::vector<Point>& hull,
    int edgeIdx,
    std::mt19937& rng,
    double thetaMaxDeg,
    Point& outUnit);

int chooseBridgeSpan(
    const std::vector<Point>& hull,
    double bridgeWidthRelTarget,
    std::mt19937& rng);

bool insertPocketOnEdge(
    std::vector<Point>& poly,
    const std::vector<Point>& hull,
    int startIdx,
    int spanEdges,
    double tau,
    const Point& directionU,
    double depth,
    int pocketVerts,
    double mouthWidthTarget = 0.0,
    PocketRecord* record = nullptr);

bool applyEdgePockets(
    std::vector<Point>& poly,
    const std::vector<Point>& hull,
    const DentTargets& targets,
    int dentCount,
    std::mt19937& rng,
    EdgePocketBuildResult* out = nullptr);
