#include "HausdorffPolygonGen.h"
#include <fstream>
#include <sstream>
#include <iomanip>
#include <filesystem>
#include <random>
#include <atomic>
#include <thread>
#include <ctime>

namespace fs = std::filesystem;

/* ================== геометрия ================== */

static double cross(const Point& a, const Point& b, const Point& c) {
    return (b.x - a.x) * (c.y - a.y) -
        (b.y - a.y) * (c.x - a.x);
}

static double polygonArea(const std::vector<Point>& p) {
    double a = 0.0;
    for (size_t i = 0; i < p.size(); ++i) {
        size_t j = (i + 1) % p.size();
        a += p[i].x * p[j].y - p[j].x * p[i].y;
    }
    return std::abs(a) * 0.5;
}

static std::vector<Point> convexHull(std::vector<Point> pts) {
    if (pts.size() <= 1) return pts;
    std::sort(pts.begin(), pts.end(), [](const Point& a, const Point& b) {
        if (a.x == b.x) return a.y < b.y;
        return a.x < b.x;
        });

    std::vector<Point> lower, upper;
    for (const auto& p : pts) {
        while (lower.size() >= 2 &&
            cross(lower[lower.size() - 2], lower.back(), p) <= 0)
            lower.pop_back();
        lower.push_back(p);
    }

    for (int i = (int)pts.size() - 1; i >= 0; --i) {
        const auto& p = pts[i];
        while (upper.size() >= 2 &&
            cross(upper[upper.size() - 2], upper.back(), p) <= 0)
            upper.pop_back();
        upper.push_back(p);
    }

    lower.pop_back();
    upper.pop_back();
    lower.insert(lower.end(), upper.begin(), upper.end());
    return lower;
}

bool PolygonGenerator::isSimple(const std::vector<Point>& poly) {
    int n = (int)poly.size();
    for (int i = 0; i < n; ++i) {
        Point a1 = poly[i];
        Point a2 = poly[(i + 1) % n];
        for (int j = i + 1; j < n; ++j) {
            if (abs(i - j) <= 1 || (i == 0 && j == n - 1)) continue;

            Point b1 = poly[j];
            Point b2 = poly[(j + 1) % n];

            auto orient = [](const Point& p, const Point& q, const Point& r) {
                return (q.y - p.y) * (r.x - q.x) -
                    (q.x - p.x) * (r.y - q.y);
                };

            auto onSeg = [](const Point& p, const Point& q, const Point& r) {
                return q.x <= std::max(p.x, r.x) &&
                    q.x >= std::min(p.x, r.x) &&
                    q.y <= std::max(p.y, r.y) &&
                    q.y >= std::min(p.y, r.y);
                };

            double o1 = orient(a1, a2, b1);
            double o2 = orient(a1, a2, b2);
            double o3 = orient(b1, b2, a1);
            double o4 = orient(b1, b2, a2);

            if (o1 * o2 < 0 && o3 * o4 < 0) return false;
            if (o1 == 0 && onSeg(a1, b1, a2)) return false;
            if (o2 == 0 && onSeg(a1, b2, a2)) return false;
            if (o3 == 0 && onSeg(b1, a1, b2)) return false;
            if (o4 == 0 && onSeg(b1, a2, b2)) return false;
        }
    }
    return true;
}

/* ================== генерация ================== */

std::vector<Point> PolygonGenerator::generateConvexPolygon(int n, double radius) {
    std::vector<Point> p(n);
    for (int i = 0; i < n; ++i) {
        double ang = 2.0 * PI * i / n;
        double r = radius * (0.7 + 0.3 * (double(rand()) / RAND_MAX));
        p[i] = { r * std::cos(ang), r * std::sin(ang) };
    }
    return p;
}

/* ================== деформация ================== */

bool PolygonGenerator::makeTargetReflex(
    HausdorffPolygon& poly,
    int idx,
    double depthCoeff)
{
    auto& v = poly.deformedMutable();
    int n = (int)v.size();

    int i0 = (idx - 1 + n) % n;
    int i1 = idx;
    int i2 = (idx + 1) % n;

    Point A = v[i0], B = v[i1], C = v[i2];

    double lenBA = hypot(A.x - B.x, A.y - B.y);
    double lenBC = hypot(C.x - B.x, C.y - B.y);
    if (lenBA < 1e-9 || lenBC < 1e-9) return false;

    double shift = std::min(lenBA, lenBC) * depthCoeff;
    Point moveDir;

    if (reflexDir == ReflexDirection::Bisector) {
        Point u{ (A.x - B.x) / lenBA, (A.y - B.y) / lenBA };
        Point w{ (C.x - B.x) / lenBC, (C.y - B.y) / lenBC };
        Point bis{ u.x + w.x, u.y + w.y };
        double l = hypot(bis.x, bis.y);
        if (l < 1e-9) return false;
        moveDir = { bis.x / l, bis.y / l };
    }
    else {
        Point G = poly.initialCentroid();
        Point d{ G.x - B.x, G.y - B.y };
        double l = hypot(d.x, d.y);
        if (l < 1e-9) return false;
        moveDir = { d.x / l, d.y / l };
    }

    v[i1].x += moveDir.x * shift;
    v[i1].y += moveDir.y * shift;
    return true;
}

void PolygonGenerator::makeReflex(
    HausdorffPolygon& poly,
    int count,
    double depthCoeff)
{
    std::mt19937 rng(std::random_device{}());
    std::uniform_int_distribution<int> idxDist(
        0, (int)poly.deformed().size() - 1
    );

    int created = 0, tries = 0;
    while (created < count && tries < 1000) {
        int idx = idxDist(rng);
        auto backup = poly.deformed();

        if (makeTargetReflex(poly, idx, depthCoeff) &&
            isSimple(poly.deformed())) {
            created++;
        }
        else {
            poly.deformedMutable() = backup;
        }
        tries++;
    }
}

/* ================== сохранение ================== */

void PolygonGenerator::saveTXT(const HausdorffPolygon& poly, const std::string& name) {
    std::ofstream out(name);
    out << poly.deformed().size() << "\n";
    for (auto& p : poly.deformed())
        out << p.x << " " << p.y << "\n";
}

void PolygonGenerator::saveSVG(const HausdorffPolygon& poly, const std::string& name) {
    std::ofstream out(name);
    out << "<svg xmlns='http://www.w3.org/2000/svg' "
        "width='800' height='800' viewBox='-150 -150 300 300'>\n";

    out << "<polygon points='";
    for (auto& p : poly.deformed())
        out << p.x << "," << p.y << " ";
    out << "' fill='none' stroke='black'/>\n";

    out << "<polygon points='";
    for (auto& p : poly.convex())
        out << p.x << "," << p.y << " ";
    out << "' fill='none' stroke='red' stroke-width='0.5'/>\n";

    out << "</svg>\n";
}


std::string PolygonGenerator::createOutputFolder(
    double diff,
    int reflex,
    int maxVertices,
    ReflexDirection direction,
    double depthCoefficient,
    const std::string& outDir)
{
    if (!outDir.empty()) {
        fs::create_directories(outDir);
        return outDir;
    }

    std::time_t t = std::time(nullptr);
    std::tm tm = *std::localtime(&t);

    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
    if (diff >= 0) oss << "_diff-" << int(diff);
    if (reflex >= 0) oss << "_reflex-" << reflex;
    if (depthCoefficient > 0) oss << "_reflexDirection-" << (direction == ReflexDirection::Bisector ? "bisector" : "centroid") << "_depthCoef-" << depthCoeff;
    oss << "_maxv-" << maxVertices;

    std::string folder = oss.str();
    fs::create_directories(folder);
    return folder;
}


bool PolygonGenerator::generateMultipleParallel(
    double targetDiff,
    int targetReflex,
    int maxVertices,
    int count,
    const std::string& outDir,
    int threads)
{
    const double epsDiff = 5.0;
    const int maxIterPerThread = 50000;

    std::string folder = createOutputFolder(targetDiff, targetReflex, maxVertices, reflexDir, depthCoeff, outDir);
    std::string baseName = folder;

    std::ofstream csv(folder + "/" + baseName + ".csv");
    csv << "index,diff,reflex,vertices,reflexDirection,depthCoef,txt,svg\n";

    std::atomic<int> generated(0);
    std::atomic<int> globalIndex(0);

    auto worker = [&](int tid) {
        std::mt19937 rng((unsigned)std::time(nullptr) + tid);

        for (int iter = 0; iter < maxIterPerThread && generated < count; ++iter) {
            int n = rng() % (maxVertices - 5) + 6;

            // convex polygon
            auto convex = generateConvexPolygon(n);
            HausdorffPolygon poly(convex);

            // deform
            makeReflex(poly, targetReflex, depthCoeff);

            if (!isSimple(poly.deformed())) continue;

            // hull diff
            auto hull = convexHull(poly.deformed());
            double diff = polygonArea(hull) - polygonArea(poly.deformed());
            int reflex = poly.reflexCount();

            bool ok = true;
            if (targetDiff >= 0) ok &= (std::abs(diff - targetDiff) <= epsDiff);
            if (targetReflex >= 0) ok &= (reflex == targetReflex);

            if (!ok) continue;

            int idx = ++globalIndex;
            if (idx > count) break;

            std::ostringstream fname;
            fname << folder << "/" << baseName << "_i-" << idx;

            std::string txtName = fname.str() + ".txt";
            std::string svgName = fname.str() + ".svg";

            saveTXT(poly, txtName);
            saveSVG(poly, svgName);

            {
                std::lock_guard<std::mutex> lock(csvMutex);
                csv << idx << "," << diff << "," << reflex << "," << n << "," << (reflexDir == ReflexDirection::Bisector ? "bisector" : "centroid") 
                    <<  "," << depthCoeff
                    << txtName << "," << svgName << "\n";
                generated++;
            }
        }
        };

    std::vector<std::thread> th;
    for (int i = 0; i < threads; i++) th.emplace_back(worker, i);
    for (auto& t : th) t.join();

    csv.close();
    return generated == count;
}
