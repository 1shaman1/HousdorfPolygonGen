#pragma once

#include "Point.h"
#include <string>
#include <vector>

void savePolygonTxt(const std::string& path, const std::vector<Point>& poly);
void savePair(
    const std::string& caseDir,
    const std::string& caseId,
    const std::vector<Point>& p0,
    const std::vector<Point>& p);
void savePreviewSvg(
    const std::string& path,
    const std::vector<Point>& p0,
    const std::vector<Point>& p);
