#include "square_sampler.h"

std::vector<Point> sampleSquare(const SquareBounds& square, int count, std::mt19937& rng) {
    std::uniform_real_distribution<double> xDist(square.xmin, square.xmax);
    std::uniform_real_distribution<double> yDist(square.ymin, square.ymax);
    std::vector<Point> pts;
    pts.reserve(static_cast<size_t>(count));
    for (int i = 0; i < count; ++i) {
        pts.push_back({xDist(rng), yDist(rng)});
    }
    return pts;
}
