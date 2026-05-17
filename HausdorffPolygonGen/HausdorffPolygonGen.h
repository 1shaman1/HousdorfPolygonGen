#pragma once

#include "batch_runner.h"

class PolygonGenerator {
public:
    int run(const BatchOptions& opts) { return runBatch(opts); }
};
