#pragma once

#include "experiment_config.h"
#include <string>

struct BatchOptions {
    std::string experiment_path;
    std::string gen_mode_override;
    std::string out_dir;
    int count_override = -1;
    int threads_override = -1;
    int seed_override = -1;
    /** If true and --seed is not set, seed_base = milliseconds since epoch. */
    bool seed_from_time = false;
    int preview_every = 0;
    bool has_square_override = false;
    SquareBounds square_override{};
    bool enrich_hausdorff = false;
    std::string python_exe = "python";
    std::string hausdorff_script_path;
    int hausdorff_raster = 80;
    int hausdorff_grid = 30;
    int hausdorff_workers = 0;
};

int runBatch(const BatchOptions& opts);
