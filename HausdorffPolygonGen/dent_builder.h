#pragma once

#include "Point.h"  // must be before Windows headers
#include <random>
#include <vector>

enum class ReflexDirection { Bisector, Centroid };

struct DentTargets {
    double depth_rel = 0.1;
    double bridge_width_rel = 0.15;
    double pocket_width_rel = 0.1;
    double area_ratio = 1.1;
    double alpha_lebedev = 0.5;
};

struct DentBuildResult {
    bool ok = false;
    int chainK = 1;
    double depthCoeff = 0.15;
};

bool pushVertexReflex(
    std::vector<Point>& poly,
    const std::vector<Point>& hull,
    int idx,
    double depthCoeff,
    ReflexDirection dir = ReflexDirection::Bisector);

int chooseChainLength(
    const std::vector<Point>& hull,
    double bridgeWidthRelTarget,
    std::mt19937& rng);

bool applyPocketAt(
    std::vector<Point>& poly,
    const std::vector<Point>& hull,
    int startIdx,
    int chainK,
    double depthCoeff,
    ReflexDirection dir = ReflexDirection::Bisector);

bool applyDents(
    std::vector<Point>& poly,
    const std::vector<Point>& hull,
    const DentTargets& targets,
    int dentCount,
    std::mt19937& rng,
    DentBuildResult* out = nullptr);
