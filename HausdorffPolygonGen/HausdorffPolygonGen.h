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
    void generateMultipleParallelRandom(int polygonCount,
     const std::string& outDir, int threadsCount);
    

private:
    static constexpr double PI = 3.14159265358979323846;

    // === генерация ===
    std::vector<Point> generateConvexPolygon(int n, double radius = 100.0);
    HausdorffPolygon createPolygon(std::mt19937& gen);

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
    static std::vector<Point> generatePoints(int count, std::mt19937& gen);
    bool isSimple(const std::vector<Point>& poly);

    // === вывод ===
    void saveTXT(const HausdorffPolygon& poly, const std::string& name);
    void saveSVG(const HausdorffPolygon& poly, const std::string& name);

    std::string createOutputFolder(
        int vertexCount,
        double araHullRatio,    
        const std::string& outDir
    );

    std::random_device rd;

    std::mutex csvMutex;
};