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

const int startPolygonAngles = 50;

/* ================== геометрия ================== */

static double cross(const Point& a, const Point& b, const Point& c) {
    return (b.x - a.x) * (c.y - a.y) -
        (b.y - a.y) * (c.x - a.x);
}

// Проверка пересечений точек
bool intersect(const Point& a, const Point& b,
               const Point& c, const Point& d)
{
    double d1 = cross(a,b,c);
    double d2 = cross(a,b,d);
    double d3 = cross(c,d,a);
    double d4 = cross(c,d,b);

    return (d1*d2 < 0) && (d3*d4 < 0);
}

double dist(Point p1, Point p2){
    return (p1.x - p2.x) * (p1.x - p2.x) +  (p1.y - p2.y) * (p1.y - p2.y);
}

/// @brief Удаление пересечний в гамильтоновом цикле алгоритмом 2-opt
/// @param p 
void removeIntersections(std::vector<Point>& p)
{
    int n = p.size();
    bool improved = true;

    while (improved)
    {
        improved = false;

        for (int i = 0; i < n; i++)
        {
            int i2 = (i + 1) % n;

            for (int j = i + 2; j < n; j++)
            {
                int j2 = (j + 1) % n;

                if (i == j2) 
                    continue;

                if (intersect(p[i], p[i2], p[j], p[j2]))
                {
                    std::reverse(p.begin() + i + 1, p.begin() + j + 1);
                    improved = true;
                }
            }
        }
    }
}

/// @brief проверка на выпуклость многоугольника
/// @param p 
/// @return 
bool isConvex(const std::vector<Point>& p)
{
    int n = p.size();
    if (n < 4) 
        return true;

    int sign = 0;

    for (int i = 0; i < n; i++)
    {
        const Point& a = p[i];
        const Point& b = p[(i + 1) % n];
        const Point& c = p[(i + 2) % n];

        double cr = cross(a, b, c);

        if (cr == 0)
            continue;

        if (sign == 0)
            sign = (cr > 0) ? 1 : -1;
        else if ((cr > 0 && sign < 0) || (cr < 0 && sign > 0))
            return false;
    }

    return true;
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

/// @brief Создаёт случайный невыпуклый многоугольник
/// @return 
HausdorffPolygon PolygonGenerator::createPolygon(std::mt19937& gen) 
{
    std::uniform_int_distribution<> countDist(5, 30);
    int n = countDist(gen);
    std::vector<Point> pointsSelected;
    while (true)
    {
        pointsSelected = generatePoints(n, gen);        
        removeIntersections(pointsSelected);
        bool isConcave = !isConvex(pointsSelected);
        if(isConcave){
            break;
        }
    }

    auto polygon = HausdorffPolygon(pointsSelected);
    
    return polygon;
}

/// @brief Генерация случайного набора точек
/// @param count 
/// @param gen 
/// @return 
std::vector<Point> PolygonGenerator::generatePoints(int count, std::mt19937& gen) {
    std::vector<Point> points;
    std::uniform_real_distribution<> coordDist(0, 1000);
    points.reserve(count);

    for (int i = 0; i < count; i++) {
        Point p;
        p.x = coordDist(gen);
        p.y = coordDist(gen);
        points.push_back(p);
    }
    
    return points;
}


/* ================== деформация ================== */

// bool PolygonGenerator::makeTargetReflex(
//     HausdorffPolygon& poly,
//     int idx,
//     double depthCoeff)
// {
//     auto& v = poly.deformedMutable();
//     int n = (int)v.size();

//     int i0 = (idx - 1 + n) % n;
//     int i1 = idx;
//     int i2 = (idx + 1) % n;

//     Point A = v[i0], B = v[i1], C = v[i2];

//     double lenBA = hypot(A.x - B.x, A.y - B.y);
//     double lenBC = hypot(C.x - B.x, C.y - B.y);
//     if (lenBA < 1e-9 || lenBC < 1e-9) return false;

//     double shift = std::min(lenBA, lenBC) * depthCoeff;
//     Point moveDir;

//     if (reflexDir == ReflexDirection::Bisector) {
//         Point u{ (A.x - B.x) / lenBA, (A.y - B.y) / lenBA };
//         Point w{ (C.x - B.x) / lenBC, (C.y - B.y) / lenBC };
//         Point bis{ u.x + w.x, u.y + w.y };
//         double l = hypot(bis.x, bis.y);
//         if (l < 1e-9) return false;
//         moveDir = { bis.x / l, bis.y / l };
//     }
//     else {
//         Point G = poly.initialCentroid();
//         Point d{ G.x - B.x, G.y - B.y };
//         double l = hypot(d.x, d.y);
//         if (l < 1e-9) return false;
//         moveDir = { d.x / l, d.y / l };
//     }

//     v[i1].x += moveDir.x * shift;
//     v[i1].y += moveDir.y * shift;
//     return true;
// }

// void PolygonGenerator::makeReflex(
//     HausdorffPolygon& poly,
//     int count,
//     double depthCoeff)
// {
//     std::mt19937 rng(std::random_device{}());
//     std::uniform_int_distribution<int> idxDist(
//         0, (int)poly.deformed().size() - 1
//     );

//     int created = 0, tries = 0;
//     while (created < count && tries < 1000) {
//         int idx = idxDist(rng);
//         auto backup = poly.deformed();

//         if (makeTargetReflex(poly, idx, depthCoeff) &&
//             isSimple(poly.deformed())) {
//             created++;
//         }
//         else {
//             poly.deformedMutable() = backup;
//         }
//         tries++;
//     }
// }

/* ================== сохранение ================== */

void PolygonGenerator::saveTXT(const HausdorffPolygon& poly, const std::string& name) {
    std::ofstream out(name);
    out << poly.convex().size() << "\n";
    for (auto& p : poly.convex())
        out << p.x << " " << p.y << "\n";
}

void PolygonGenerator::saveSVG(const HausdorffPolygon& poly, const std::string& name) {
    std::ofstream out(name);
    out << "<svg xmlns='http://www.w3.org/2000/svg' "
        "width='1200' height='1200' viewBox='-100 -100 1500 1500'>\n";

    out << "<polygon points='";
    for (auto& p : poly.convex())
        out << p.x << "," << p.y << " ";
    out << "' fill='none' stroke='black'/>\n";

    out << "<polygon points='";
    for (auto& p : poly.convexHull())
        out << p.x << "," << p.y << " ";
    out << "' fill='none' stroke='red' stroke-width='0.5'/>\n";

    out << "</svg>\n";
}


std::string PolygonGenerator::createOutputFolder(
    int vertexCount,
    double araHullRatio,    
    const std::string& outDir)
{
    if (!outDir.empty()) {
        fs::create_directories(outDir);
        return outDir;
    }

    std::time_t t = std::time(nullptr);
    std::tm tm;
    localtime_s(&tm, &t);

    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
    if (araHullRatio >= 0) oss << "_diff-" << araHullRatio;
    if (vertexCount >= 0) oss << "_reflex-" << vertexCount;

    std::string folder = oss.str();
    fs::create_directories(folder);
    return folder;
}


void PolygonGenerator::generateMultipleParallelRandom(int polygonCount,
     const std::string& outDir, int threadsCount){
    
    const int maxIterPerThread = 50;
    std::atomic<int> generated(0);
    std::atomic<int> globalIndex(0);
    
    std::time_t t = std::time(nullptr);
    std::tm tm;
    localtime_s(&tm, &t);

    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
    auto mainFolderName = oss.str();
    fs::create_directories(mainFolderName);
    std::ofstream csv(mainFolderName + "/metadata.csv");
    csv << "path;vertexCount;reflexCount;area;areaHullRatio\n";

    auto worker = [&](int tid) {
        std::mt19937 rng((unsigned)std::time(nullptr) + tid);
        for (int iter = 0; iter < maxIterPerThread && generated < polygonCount; ++iter) {
            auto polygon = createPolygon(rng);
            std::string folder = createOutputFolder(polygon.convex().size(), polygon.areaHullRatio(), outDir);
            std::string baseName = folder;
            ++generated;
            saveTXT(polygon, baseName + "/polygon.txt");
            saveSVG(polygon, baseName + "/polygon.svg");

            {
                std::lock_guard<std::mutex> lock(csvMutex);
                csv << baseName << "/polygon.txt;" 
                    << polygon.convex().size() << ";"
                    << polygon.reflexCount() << ";"
                    << polygon.area() << ";"
                    << polygon.areaHullRatio() << "\n";
            }
        }
    };

    std::vector<std::thread> th;
    for (int i = 0; i < threadsCount; i++) th.emplace_back(worker, i);
    for (auto& t : th) t.join();

    csv.close();
}
