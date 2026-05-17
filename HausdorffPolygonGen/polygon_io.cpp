#include "polygon_io.h"
#include <filesystem>
#include <fstream>

namespace fs = std::filesystem;

void savePolygonTxt(const std::string& path, const std::vector<Point>& poly) {
    std::ofstream out(path);
    out << poly.size() << "\n";
    for (const auto& p : poly) {
        out << p.x << " " << p.y << "\n";
    }
}

void savePair(
    const std::string& caseDir,
    const std::string& caseId,
    const std::vector<Point>& p0,
    const std::vector<Point>& p)
{
    fs::create_directories(caseDir);
    savePolygonTxt(caseDir + "/" + caseId + "_polygon_convex.txt", p0);
    savePolygonTxt(caseDir + "/" + caseId + "_polygon_nonconvex.txt", p);
}

void savePreviewSvg(
    const std::string& path,
    const std::vector<Point>& p0,
    const std::vector<Point>& p)
{
    std::ofstream out(path);
    out << "<svg xmlns='http://www.w3.org/2000/svg' "
           "width='1200' height='1200' viewBox='-50 -50 1100 1100'>\n";
    out << "<polygon points='";
    for (const auto& pt : p) out << pt.x << "," << pt.y << " ";
    out << "' fill='lightblue' fill-opacity='0.3' stroke='blue'/>\n";
    out << "<polygon points='";
    for (const auto& pt : p0) out << pt.x << "," << pt.y << " ";
    out << "' fill='none' stroke='red' stroke-width='2'/>\n";
    out << "</svg>\n";
}
