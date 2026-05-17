#pragma once

#include "Point.h"
#include "experiment_config.h"
#include "metric_calculator.h"
#include "sweep_planner.h"
#include <vector>

struct GeneratedCase {
    std::vector<Point> p0;
    std::vector<Point> p;
    PolygonMetrics metrics;
    int chainK = 1;
    bool ok = false;
};

GeneratedCase generateCase(const ExperimentConfig& cfg, const GenJob& job);
