#pragma once

#include <string>

struct HausdorffEnrichConfig {
    std::string python_exe = "python";
    std::string script_path;
    int raster_steps = 50;
    int grid_steps = 20;
    int workers = 0;
};

/// Запускает tools/enrich_metadata_hausdorff.py: grid+Shor и УЛЛ → metadata.csv.
/// Возвращает 0 при успехе, иначе код выхода Python или -1 если скрипт не найден.
int runHausdorffMetadataEnrichment(
    const std::string& runDir,
    const HausdorffEnrichConfig& cfg);
