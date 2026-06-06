#pragma once

#include "square_sampler.h"
#include <map>
#include <string>
#include <vector>

struct SweepAxis {
    double min = 0.0;
    double max = 0.0;
    double step = 0.0;
};

struct ExperimentConfig {
    SquareBounds square;
    /** Размер окна одной кляксы; для каждой кляксы окно заново случайно в square. */
    SquareBounds blob_square;
    bool has_blob_square = false;
    std::string gen_mode = "pockets";
    int points_per_hull = 30;
    int dent_count = 2;
    int seed_base = 42;
    /** When true, runBatch replaces seed_base with a value from the system clock. */
    bool seed_from_time = false;
    int count = 1000;
    int replicate = 1;
    int threads = 8;
    std::string sweep_mode = "one_at_a_time";
    std::map<std::string, SweepAxis> sweeps;
    int preview_every = 0;
    int min_hull_vertices = 4;
    int max_attempts = 48;
};

bool loadExperimentConfig(const std::string& path, ExperimentConfig& out);
bool saveExperimentConfig(const std::string& path, const ExperimentConfig& cfg);
ExperimentConfig defaultExperimentConfig();

std::vector<double> sweepLevels(const SweepAxis& axis);
