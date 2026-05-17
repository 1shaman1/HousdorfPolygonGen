#pragma once

#include "case_generator.h"
#include "experiment_config.h"
#include "sweep_planner.h"

GeneratedCase generateRandomHullCase(const ExperimentConfig& cfg, const GenJob& job);
