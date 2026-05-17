#pragma once

#include "Point.h"
#include "dent_builder.h"
#include "experiment_config.h"
#include <string>
#include <vector>

struct GenTargets {
    double depth_rel = 0.1;
    double bridge_width_rel = 0.15;
    double pocket_width_rel = 0.1;
    double area_ratio = 1.1;
    double alpha_lebedev = 0.5;
};

struct GenJob {
    std::string case_id;
    unsigned seed = 0;
    int replicate = 0;
    std::string sweep_axis;
    double sweep_level = 0.0;
    GenTargets targets;
};

std::string makeCaseId(int seedBase, const std::string& axis, double level, int replicate);
std::vector<GenJob> planJobs(const ExperimentConfig& cfg);
