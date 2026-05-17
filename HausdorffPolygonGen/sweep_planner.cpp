#include "sweep_planner.h"
#include <cmath>
#include <iomanip>
#include <random>
#include <sstream>

namespace {

GenTargets medians(const ExperimentConfig& cfg) {
    GenTargets t;
    auto pick = [&](const char* name, double& slot) {
        auto it = cfg.sweeps.find(name);
        if (it == cfg.sweeps.end()) return;
        slot = (it->second.min + it->second.max) * 0.5;
    };
    pick("depth_rel", t.depth_rel);
    pick("bridge_width_rel", t.bridge_width_rel);
    pick("pocket_width_rel", t.pocket_width_rel);
    pick("area_ratio", t.area_ratio);
    pick("alpha_lebedev", t.alpha_lebedev);
    return t;
}

void setTarget(GenTargets& t, const std::string& axis, double v) {
    if (axis == "depth_rel") t.depth_rel = v;
    else if (axis == "bridge_width_rel") t.bridge_width_rel = v;
    else if (axis == "pocket_width_rel") t.pocket_width_rel = v;
    else if (axis == "area_ratio") t.area_ratio = v;
    else if (axis == "alpha_lebedev") t.alpha_lebedev = v;
}

GenTargets lhsSample(
    const ExperimentConfig& cfg,
    std::mt19937& rng,
    const std::vector<std::string>& axisNames)
{
    GenTargets t = medians(cfg);
    std::uniform_real_distribution<double> u01(0.0, 1.0);
    for (const auto& name : axisNames) {
        auto it = cfg.sweeps.find(name);
        if (it == cfg.sweeps.end()) continue;
        const double v = it->second.min + u01(rng) * (it->second.max - it->second.min);
        setTarget(t, name, v);
    }
    return t;
}

}  // namespace

std::string makeCaseId(int seedBase, const std::string& axis, double level, int replicate) {
    std::ostringstream oss;
    oss << "s" << seedBase << "_" << axis << "_"
        << std::fixed << std::setprecision(4) << level << "_r" << replicate;
    return oss.str();
}

int plannedJobCount(const ExperimentConfig& cfg) {
    int levels = 0;
    for (const auto& kv : cfg.sweeps)
        levels += static_cast<int>(sweepLevels(kv.second).size());
    if (levels == 0) levels = 1;
    return levels * std::max(1, cfg.replicate);
}

std::vector<GenJob> planJobs(const ExperimentConfig& cfg) {
    std::vector<GenJob> jobs;
    int jobLimit = cfg.count;
    if (jobLimit <= 0) jobLimit = plannedJobCount(cfg);
    if (jobLimit <= 0) return jobs;

    const GenTargets base = medians(cfg);
    std::vector<std::pair<std::string, std::vector<double>>> axes;
    for (const auto& kv : cfg.sweeps) {
        axes.emplace_back(kv.first, sweepLevels(kv.second));
    }
    if (axes.empty()) {
        GenJob j;
        j.targets = base;
        j.seed = static_cast<unsigned>(cfg.seed_base);
        j.replicate = 0;
        j.sweep_axis = "none";
        j.sweep_level = 0.0;
        j.case_id = makeCaseId(cfg.seed_base, "none", 0.0, 0);
        jobs.push_back(j);
        return jobs;
    }

    if (cfg.sweep_mode == "full") {
        std::vector<GenTargets> grid{base};
        for (const auto& ax : axes) {
            std::vector<GenTargets> next;
            for (const auto& g : grid) {
                for (double lvl : ax.second) {
                    GenTargets t = g;
                    setTarget(t, ax.first, lvl);
                    next.push_back(t);
                }
            }
            grid.swap(next);
        }
        for (int i = 0; i < jobLimit; ++i) {
            GenJob j;
            j.targets = grid[static_cast<size_t>(i % grid.size())];
            j.replicate = i % std::max(1, cfg.replicate);
            j.seed = static_cast<unsigned>(cfg.seed_base + i);
            j.sweep_axis = "full";
            j.sweep_level = 0.0;
            j.case_id = makeCaseId(cfg.seed_base, "full", static_cast<double>(i), i);
            jobs.push_back(j);
        }
        return jobs;
    }

    if (cfg.sweep_mode == "lhs") {
        std::mt19937 rng(static_cast<unsigned>(cfg.seed_base));
        std::vector<std::string> names;
        for (const auto& ax : axes) names.push_back(ax.first);
        for (int i = 0; i < jobLimit; ++i) {
            GenJob j;
            j.targets = lhsSample(cfg, rng, names);
            j.replicate = i % std::max(1, cfg.replicate);
            j.seed = static_cast<unsigned>(cfg.seed_base + i);
            j.sweep_axis = "lhs";
            j.sweep_level = 0.0;
            j.case_id = makeCaseId(cfg.seed_base, "lhs", static_cast<double>(i), i);
            jobs.push_back(j);
        }
        return jobs;
    }

    // one_at_a_time and target_bins share planner; bins use actual columns in CSV
    size_t idx = 0;
    while (static_cast<int>(jobs.size()) < jobLimit) {
        for (const auto& ax : axes) {
            for (double lvl : ax.second) {
                if (static_cast<int>(jobs.size()) >= jobLimit) break;
                for (int rep = 0; rep < std::max(1, cfg.replicate); ++rep) {
                    if (static_cast<int>(jobs.size()) >= jobLimit) break;
                    GenJob j;
                    j.targets = base;
                    setTarget(j.targets, ax.first, lvl);
                    j.replicate = rep;
                    j.seed = static_cast<unsigned>(cfg.seed_base + static_cast<int>(idx));
                    j.sweep_axis = ax.first;
                    j.sweep_level = lvl;
                    j.case_id = makeCaseId(cfg.seed_base, ax.first, lvl, static_cast<int>(idx));
                    jobs.push_back(j);
                    ++idx;
                }
            }
        }
        if (cfg.sweep_mode != "target_bins") break;
    }

    return jobs;
}
