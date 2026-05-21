#pragma once

#include "case_generator.h"
#include "experiment_config.h"
#include "sweep_planner.h"

GeneratedCase generateRandomHullCase(const ExperimentConfig& cfg, const GenJob& job);

/**
 * Several small convex blobs (scatter + Graham hull, 4–10 hull vertices) in the square;
 * their outer union boundary is stitched into one simple non-convex polygon,
 * with gap bridges to the nearest vertex of another blob when needed.
 */
GeneratedCase generateConvexBlobsCase(const ExperimentConfig& cfg, const GenJob& job);
