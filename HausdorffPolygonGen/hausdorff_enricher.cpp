#include "hausdorff_enricher.h"
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <sstream>

namespace fs = std::filesystem;

namespace {

std::string shellQuote(const std::string& s) {
#if defined(_WIN32)
    std::string out = "\"";
    for (char c : s) {
        if (c == '"') out += "\\\"";
        else out += c;
    }
    out += '"';
    return out;
#else
    std::string out = "'";
    for (char c : s) {
        if (c == '\'') out += "'\\''";
        else out += c;
    }
    out += '\'';
    return out;
#endif
}

fs::path resolveEnrichScript(const std::string& configured) {
    if (!configured.empty() && fs::is_regular_file(configured)) {
        return configured;
    }
    const fs::path rel = "tools/enrich_metadata_hausdorff.py";
    const fs::path candidates[] = {
        fs::current_path() / rel,
        fs::current_path() / "HousdorfPolygonGen" / rel,
        fs::current_path().parent_path() / "HousdorfPolygonGen" / rel,
    };
    for (const auto& p : candidates) {
        if (fs::is_regular_file(p)) return fs::weakly_canonical(p);
    }
    return {};
}

}  // namespace

int runHausdorffMetadataEnrichment(
    const std::string& runDir,
    const HausdorffEnrichConfig& cfg)
{
    const fs::path script = resolveEnrichScript(cfg.script_path);
    if (script.empty()) {
        std::cerr << "hausdorff enrich: tools/enrich_metadata_hausdorff.py not found\n";
        return -1;
    }
    if (!fs::is_directory(runDir)) {
        std::cerr << "hausdorff enrich: run dir not found: " << runDir << '\n';
        return -1;
    }

    std::ostringstream cmd;
    cmd << shellQuote(cfg.python_exe) << ' '
        << shellQuote(script.string()) << " --run-dir "
        << shellQuote(runDir) << " --raster " << cfg.raster_steps << " --grid "
        << cfg.grid_steps;
    if (cfg.workers > 0) {
        cmd << " --workers " << cfg.workers;
    }

    std::cout << "Hausdorff enrich: " << cmd.str() << '\n';
    const int rc = std::system(cmd.str().c_str());
    if (rc != 0) {
        std::cerr << "hausdorff enrich failed (exit " << rc << ")\n";
    }
    return rc;
}
