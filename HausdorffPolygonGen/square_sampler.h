#pragma once

#include "Point.h"
#include <random>
#include <vector>

struct SquareBounds {
    double xmin = 0.0;
    double xmax = 1000.0;
    double ymin = 0.0;
    double ymax = 1000.0;
};

std::vector<Point> sampleSquare(const SquareBounds& square, int count, std::mt19937& rng);
