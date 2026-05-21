#include "experiment_config.h"
#include "polygon_limits.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <regex>
#include <sstream>

namespace {

std::string trim(const std::string& s) {
    size_t a = s.find_first_not_of(" \t\r\n");
    if (a == std::string::npos) return "";
    size_t b = s.find_last_not_of(" \t\r\n");
    return s.substr(a, b - a + 1);
}

bool parseDouble(const std::string& s, double& v) {
    try {
        size_t pos = 0;
        v = std::stod(s, &pos);
        return pos > 0;
    } catch (...) {
        return false;
    }
}

bool parseInt(const std::string& s, int& v) {
    try {
        size_t pos = 0;
        v = std::stoi(s, &pos);
        return pos > 0;
    } catch (...) {
        return false;
    }
}

std::string extractObjectBody(const std::string& text, const std::string& key) {
    const std::regex re("\"" + key + "\"\\s*:\\s*\\{");
    std::smatch m;
    if (!std::regex_search(text, m, re)) return "";
    const size_t start = m.position() + m.length() - 1;
    int depth = 0;
    for (size_t i = start; i < text.size(); ++i) {
        if (text[i] == '{') ++depth;
        else if (text[i] == '}') {
            --depth;
            if (depth == 0) return text.substr(start + 1, i - start - 1);
        }
    }
    return "";
}

bool parseAxisBody(const std::string& body, SweepAxis& axis) {
    static const std::regex kv("\"(min|max|step)\"\\s*:\\s*([-+0-9.eE]+)");
    std::sregex_iterator it(body.begin(), body.end(), kv);
    std::sregex_iterator end;
    bool any = false;
    for (; it != end; ++it) {
        double v = 0.0;
        if (!parseDouble((*it)[2].str(), v)) continue;
        const std::string k = (*it)[1].str();
        if (k == "min") axis.min = v;
        else if (k == "max") axis.max = v;
        else if (k == "step") axis.step = v;
        any = true;
    }
    return any && axis.step > 0.0;
}

}  // namespace

std::vector<double> sweepLevels(const SweepAxis& axis) {
    std::vector<double> levels;
    if (axis.step <= 0.0) return levels;
    for (double v = axis.min; v <= axis.max + 1e-9; v += axis.step)
        levels.push_back(v);
    if (levels.empty()) levels.push_back(axis.min);
    return levels;
}

bool parseSquareBody(const std::string& body, SquareBounds& out) {
    if (body.empty()) return false;
    static const std::regex sq("\"(xmin|xmax|ymin|ymax)\"\\s*:\\s*([-+0-9.eE]+)");
    bool any = false;
    std::sregex_iterator sqIt(body.begin(), body.end(), sq);
    for (; sqIt != std::sregex_iterator(); ++sqIt) {
        double v = 0.0;
        if (!parseDouble((*sqIt)[2].str(), v)) continue;
        any = true;
        const std::string k = (*sqIt)[1].str();
        if (k == "xmin") out.xmin = v;
        else if (k == "xmax") out.xmax = v;
        else if (k == "ymin") out.ymin = v;
        else if (k == "ymax") out.ymax = v;
    }
    return any;
}

ExperimentConfig defaultExperimentConfig() {
    ExperimentConfig c;
    c.square = {0, 1000, 0, 1000};
    c.points_per_hull = 30;
    c.dent_count = 2;
    c.seed_base = 42;
    c.count = 1000;
    c.max_attempts = 48;
    c.threads = 8;
    c.sweep_mode = "one_at_a_time";
    c.sweeps = {
        {"depth_rel", {0.02, 0.35, 0.03}},
        {"bridge_width_rel", {0.05, 0.50, 0.05}},
        {"pocket_width_rel", {0.03, 0.40, 0.04}},
        {"area_ratio", {1.02, 1.40, 0.04}},
        {"alpha_lebedev", {0.10, 1.20, 0.10}},
    };
    return c;
}

bool loadExperimentConfig(const std::string& path, ExperimentConfig& out) {
    std::ifstream in(path);
    if (!in) return false;
    std::ostringstream ss;
    ss << in.rdbuf();
    const std::string text = ss.str();

    out = defaultExperimentConfig();

    static const std::regex numField(
        "\"(points_per_hull|dent_count|seed_base|count|replicate|threads|preview_every|min_hull_vertices|max_attempts)\"\\s*:\\s*([-+0-9]+)");
    std::sregex_iterator it(text.begin(), text.end(), numField);
    std::sregex_iterator end;
    for (; it != end; ++it) {
        int v = 0;
        if (!parseInt((*it)[2].str(), v)) continue;
        const std::string k = (*it)[1].str();
        if (k == "points_per_hull") out.points_per_hull = std::min(v, kMaxPolygonVertices);
        else if (k == "dent_count") out.dent_count = v;
        else if (k == "seed_base") out.seed_base = v;
        else if (k == "count") out.count = v;
        else if (k == "replicate") out.replicate = std::max(1, v);
        else if (k == "threads") out.threads = v;
        else if (k == "preview_every") out.preview_every = v;
        else if (k == "min_hull_vertices") out.min_hull_vertices = v;
        else if (k == "max_attempts") out.max_attempts = std::max(1, v);
    }

    static const std::regex strField("\"(sweep_mode|gen_mode)\"\\s*:\\s*\"([^\"]+)\"");
    std::sregex_iterator it2(text.begin(), text.end(), strField);
    for (; it2 != end; ++it2) {
        const std::string k = (*it2)[1].str();
        if (k == "sweep_mode") out.sweep_mode = (*it2)[2].str();
        else if (k == "gen_mode") out.gen_mode = (*it2)[2].str();
    }

    SquareBounds sqParsed;
    if (parseSquareBody(extractObjectBody(text, "square"), sqParsed)) {
        out.square = sqParsed;
    }
    if (parseSquareBody(extractObjectBody(text, "blob_square"), sqParsed)) {
        out.blob_square = sqParsed;
        out.has_blob_square = true;
    }

    out.sweeps.clear();
    static const char* axisNames[] = {
        "depth_rel", "bridge_width_rel", "pocket_width_rel", "area_ratio", "alpha_lebedev"};
    for (const char* name : axisNames) {
        const std::string body = extractObjectBody(text, name);
        if (body.empty()) continue;
        SweepAxis axis;
        if (parseAxisBody(body, axis)) out.sweeps[name] = axis;
    }
    if (out.sweeps.empty()) out.sweeps = defaultExperimentConfig().sweeps;

    out.points_per_hull = std::min(out.points_per_hull, kMaxPolygonVertices);

    return true;
}

bool saveExperimentConfig(const std::string& path, const ExperimentConfig& cfg) {
    std::ofstream out(path);
    if (!out) return false;
    out << "{\n";
    out << "  \"square\": { \"xmin\": " << cfg.square.xmin << ", \"xmax\": " << cfg.square.xmax
        << ", \"ymin\": " << cfg.square.ymin << ", \"ymax\": " << cfg.square.ymax << " },\n";
    if (cfg.has_blob_square) {
        out << "  \"blob_square\": { \"xmin\": " << cfg.blob_square.xmin
            << ", \"xmax\": " << cfg.blob_square.xmax << ", \"ymin\": " << cfg.blob_square.ymin
            << ", \"ymax\": " << cfg.blob_square.ymax << " },\n";
    }
    out << "  \"gen_mode\": \"" << cfg.gen_mode << "\",\n";
    out << "  \"points_per_hull\": " << cfg.points_per_hull << ",\n";
    out << "  \"min_hull_vertices\": " << cfg.min_hull_vertices << ",\n";
    out << "  \"dent_count\": " << cfg.dent_count << ",\n";
    out << "  \"seed_base\": " << cfg.seed_base << ",\n";
    out << "  \"sweep_mode\": \"" << cfg.sweep_mode << "\",\n";
    out << "  \"count\": " << cfg.count << ",\n";
    out << "  \"replicate\": " << cfg.replicate << ",\n";
    out << "  \"max_attempts\": " << cfg.max_attempts << ",\n";
    out << "  \"threads\": " << cfg.threads << ",\n";
    out << "  \"sweeps\": {\n";
    bool first = true;
    for (const auto& kv : cfg.sweeps) {
        if (!first) out << ",\n";
        first = false;
        out << "    \"" << kv.first << "\": { \"min\": " << kv.second.min << ", \"max\": "
            << kv.second.max << ", \"step\": " << kv.second.step << " }";
    }
    out << "\n  }\n}\n";
    return true;
}
