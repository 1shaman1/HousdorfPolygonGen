#pragma once

#include "edge_pocket_builder.h"
#include <random>
#include <string>
#include <vector>

/// Случайные карманы: контролируется только одна метрика, остальное — случайно.
bool applyFreeRandomPockets(
    std::vector<Point>& poly,
    const std::vector<Point>& hull,
    const std::string& fixedAxis,
    double fixedValue,
    std::mt19937& rng,
    EdgePocketBuildResult* out = nullptr);
