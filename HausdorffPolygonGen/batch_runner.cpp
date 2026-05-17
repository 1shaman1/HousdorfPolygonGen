#include "batch_runner.h"
#include "case_generator.h"
#include "hausdorff_enricher.h"
#include "polygon_io.h"
#include "sweep_planner.h"
#include <chrono>
#include <iostream>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <thread>
#include <vector>

namespace fs = std::filesystem;

namespace {

std::string timestampRunDir() {
    const auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
#if defined(_WIN32)
    localtime_s(&tm, &t);
#else
    localtime_r(&t, &tm);
#endif
    std::ostringstream oss;
    oss << "run_" << std::put_time(&tm, "%Y%m%d_%H%M%S");
    return oss.str();
}

std::string metadataHeader() {
    return "case_id;seed;dent_count;n_hull;"
           "depth_rel_target;depth_rel_actual;depth_rel_err;"
           "bridge_width_rel_target;bridge_width_rel_actual;bridge_width_rel_err;"
           "pocket_width_rel_target;pocket_width_rel_actual;pocket_width_rel_err;"
           "area_ratio_target;area_ratio_actual;area_ratio_err;"
           "alpha_lebedev_target;alpha_proxy_actual;alpha_proxy_err;"
           "alpha_lebedev_full;reflex_count;sweep_axis;sweep_level;replicate\n";
}

std::string metadataRow(
    const GenJob& job,
    const ExperimentConfig& cfg,
    const GeneratedCase& gc)
{
    std::ostringstream row;
    row << std::fixed << std::setprecision(6);
    const auto err = [](double t, double a) { return std::abs(t - a); };
    row << job.case_id << ';' << job.seed << ';' << cfg.dent_count << ';' << gc.metrics.n_hull << ';'
        << job.targets.depth_rel << ';' << gc.metrics.depth_rel << ';'
        << err(job.targets.depth_rel, gc.metrics.depth_rel) << ';'
        << job.targets.bridge_width_rel << ';' << gc.metrics.bridge_width_rel << ';'
        << err(job.targets.bridge_width_rel, gc.metrics.bridge_width_rel) << ';'
        << job.targets.pocket_width_rel << ';' << gc.metrics.pocket_width_rel << ';'
        << err(job.targets.pocket_width_rel, gc.metrics.pocket_width_rel) << ';'
        << job.targets.area_ratio << ';' << gc.metrics.area_ratio << ';'
        << err(job.targets.area_ratio, gc.metrics.area_ratio) << ';'
        << job.targets.alpha_lebedev << ';' << gc.metrics.alpha_proxy << ';'
        << err(job.targets.alpha_lebedev, gc.metrics.alpha_proxy) << ';'
        << gc.metrics.alpha_lebedev << ';'
        << gc.metrics.reflex_count << ';'
        << job.sweep_axis << ';' << job.sweep_level << ';' << job.replicate << '\n';
    return row.str();
}

void mergeShards(const std::string& runDir, int threads) {
    std::ofstream merged(runDir + "/metadata.csv");
    merged << metadataHeader();
    for (int t = 0; t < threads; ++t) {
        const std::string shardPath = runDir + "/shard_" + std::to_string(t) + ".csv";
        std::ifstream shard(shardPath);
        if (!shard) continue;
        std::string line;
        bool first = true;
        while (std::getline(shard, line)) {
            if (first) {
                first = false;
                continue;
            }
            if (!line.empty()) merged << line << '\n';
        }
    }
}

}  // namespace

int runBatch(const BatchOptions& opts) {
    ExperimentConfig cfg = defaultExperimentConfig();
    if (!opts.experiment_path.empty()) {
        if (!loadExperimentConfig(opts.experiment_path, cfg)) {
            return 1;
        }
    }
    if (opts.count_override > 0) cfg.count = opts.count_override;
    if (opts.threads_override > 0) cfg.threads = opts.threads_override;
    if (opts.seed_override >= 0) cfg.seed_base = opts.seed_override;
    if (opts.preview_every > 0) cfg.preview_every = opts.preview_every;
    if (opts.has_square_override) cfg.square = opts.square_override;

    int threads = cfg.threads;
    if (threads <= 0) {
        threads = static_cast<int>(std::thread::hardware_concurrency());
        if (threads <= 0) threads = 2;
    }

    const std::string runDir = opts.out_dir.empty() ? timestampRunDir() : opts.out_dir;
    fs::create_directories(runDir);
    saveExperimentConfig(runDir + "/experiment.json", cfg);
    if (!opts.experiment_path.empty()) {
        try {
            fs::copy_file(opts.experiment_path, runDir + "/experiment_source.json", fs::copy_options::overwrite_existing);
        } catch (...) {
        }
    }

    const std::vector<GenJob> jobs = planJobs(cfg);
    if (jobs.empty()) return 2;

    std::vector<std::thread> workers;
    workers.reserve(static_cast<size_t>(threads));
    for (int tid = 0; tid < threads; ++tid) {
        workers.emplace_back([&, tid]() {
            std::ofstream shard(runDir + "/shard_" + std::to_string(tid) + ".csv");
            shard << metadataHeader();
            for (size_t i = static_cast<size_t>(tid); i < jobs.size(); i += static_cast<size_t>(threads)) {
                const GenJob& job = jobs[i];
                GeneratedCase gc = generateCase(cfg, job);
                if (!gc.ok) continue;

                const std::string caseDir = runDir + "/" + job.case_id;
                savePair(caseDir, job.case_id, gc.p0, gc.p);

                if (cfg.preview_every > 0 && (static_cast<int>(i) % cfg.preview_every) == 0) {
                    savePreviewSvg(caseDir + "/" + job.case_id + ".svg", gc.p0, gc.p);
                }
                shard << metadataRow(job, cfg, gc);
            }
        });
    }
    for (auto& th : workers) th.join();

    mergeShards(runDir, threads);
    for (int t = 0; t < threads; ++t) {
        const std::string shardPath = runDir + "/shard_" + std::to_string(t) + ".csv";
        std::error_code ec;
        fs::remove(shardPath, ec);
    }

    std::cout << "Generated batch in " << runDir << " (" << jobs.size() << " jobs planned)\n";

    if (opts.enrich_hausdorff) {
        HausdorffEnrichConfig he{};
        he.python_exe = opts.python_exe;
        he.script_path = opts.hausdorff_script_path;
        he.raster_steps = opts.hausdorff_raster;
        he.grid_steps = opts.hausdorff_grid;
        he.workers = opts.hausdorff_workers;
        const int erc = runHausdorffMetadataEnrichment(runDir, he);
        if (erc != 0) {
            return erc > 0 ? erc : 3;
        }
    }

    return 0;
}
