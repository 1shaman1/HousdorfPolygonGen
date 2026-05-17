#pragma once

#include "Point.h"
#include <vector>

// Full non-convexity measure alpha(M) from Lebedev (2007):
// alpha(M) = sup_{z not in M} alpha_M(z), where alpha_M(z) is the maximal opening
// angle of the cone from z to conv(Omega_M(z)), Omega_M(z) = closest points on dM.

double computeAlphaLebedev(const std::vector<Point>& poly);
