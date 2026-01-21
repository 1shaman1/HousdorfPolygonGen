#pragma once

#include <string>
#include <mutex>
#include <vector>
#include "HausdorffPolygon.h"

enum class ReflexDirection {
    Bisector,
    Centroid
};

class PolygonGenerator {
public:
    ReflexDirection reflexDir = ReflexDirection::Bisector;
    double depthCoeff = 0.3;

    bool generateMultipleParallel(
        double targetDiff,
        int targetReflex,
        int maxVertices,
        int count,
        const std::string& outDir,
        int threads
    );

private:
    static constexpr double PI = 3.14159265358979323846;

    // === генерация ===
    std::vector<Point> generateConvexPolygon(int n, double radius = 100.0);

    // === деформации ===
    bool makeTargetReflex(
        HausdorffPolygon& poly,
        int idx,
        double depthCoeff
    );

    void makeReflex(
        HausdorffPolygon& poly,
        int count,
        double depthCoeff
    );

    // === геометрия ===
    bool isSimple(const std::vector<Point>& poly);

    // === вывод ===
    void saveTXT(const HausdorffPolygon& poly, const std::string& name);
    void saveSVG(const HausdorffPolygon& poly, const std::string& name);

    std::string createOutputFolder(
        double diff,
        int reflex,
        int maxVertices,
        ReflexDirection direction,
        double depthCoefficient,
        const std::string& outDir
    );

    std::mutex csvMutex;
};