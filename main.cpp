#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include "HausdorffPolygonGen/HausdorffPolygonGen.h"
#include "HausdorffPolygonGen/experiment_config.h"

static void printHelp() {
    std::cout << "HousdorfPolygonGen — генератор пар P0/P для бенчмарка Hausdorff\n\n";
    std::cout << "Usage:\n";
    std::cout << "  polygon_gen --experiment <file.json> [options]\n";
    std::cout << "  polygon_gen --count <N> --threads <T> [options]\n\n";
    std::cout << "Options:\n";
    std::cout << "  --experiment <path>     experiment.json (sweeps, square, dent_count)\n";
    std::cout << "  --count <N>             override number of cases\n";
    std::cout << "  --threads <T>           worker threads (default: from experiment or HW)\n";
    std::cout << "  --out-dir <DIR>         output run_* folder (default: run_<timestamp>)\n";
    std::cout << "  --seed <S>              override seed_base\n";
    std::cout << "  --square xmin xmax ymin ymax\n";
    std::cout << "  --preview-every <K>     write SVG every K-th case (0 = off)\n";
    std::cout << "  --enrich-hausdorff      grid+Shor и УЛЛ → metadata.csv (Python)\n";
    std::cout << "  --python <exe>          интерпретатор (default: python)\n";
    std::cout << "  --hausdorff-raster <N>  шаги растеризации (default: 50)\n";
    std::cout << "  --hausdorff-grid <N>    шаги сетки (default: 20)\n";
    std::cout << "  --hausdorff-workers <W> потоки enrich (0 = auto)\n";
    std::cout << "  --help, -h\n";
}

int main(int argc, char** argv) {
    if (argc == 1) {
        printHelp();
        return 0;
    }

    BatchOptions opts;
    ExperimentConfig cliCfg = defaultExperimentConfig();
    bool haveSquare = false;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            printHelp();
            return 0;
        }
        if (arg == "--experiment" && i + 1 < argc) {
            opts.experiment_path = argv[++i];
        } else if (arg == "--count" && i + 1 < argc) {
            opts.count_override = std::atoi(argv[++i]);
        } else if (arg == "--threads" && i + 1 < argc) {
            opts.threads_override = std::atoi(argv[++i]);
        } else if (arg == "--out-dir" && i + 1 < argc) {
            opts.out_dir = argv[++i];
        } else if (arg == "--seed" && i + 1 < argc) {
            opts.seed_override = std::atoi(argv[++i]);
        } else if (arg == "--preview-every" && i + 1 < argc) {
            opts.preview_every = std::atoi(argv[++i]);
        } else if (arg == "--enrich-hausdorff") {
            opts.enrich_hausdorff = true;
        } else if (arg == "--python" && i + 1 < argc) {
            opts.python_exe = argv[++i];
        } else if (arg == "--hausdorff-raster" && i + 1 < argc) {
            opts.hausdorff_raster = std::atoi(argv[++i]);
        } else if (arg == "--hausdorff-grid" && i + 1 < argc) {
            opts.hausdorff_grid = std::atoi(argv[++i]);
        } else if (arg == "--hausdorff-workers" && i + 1 < argc) {
            opts.hausdorff_workers = std::atoi(argv[++i]);
        } else if (arg == "--square" && i + 4 < argc) {
            cliCfg.square.xmin = std::atof(argv[++i]);
            cliCfg.square.xmax = std::atof(argv[++i]);
            cliCfg.square.ymin = std::atof(argv[++i]);
            cliCfg.square.ymax = std::atof(argv[++i]);
            haveSquare = true;
        } else {
            std::cerr << "Unknown argument: " << arg << "\n";
            return 1;
        }
    }

    if (opts.experiment_path.empty() && opts.count_override < 0) {
        opts.count_override = 100;
    }

    if (haveSquare) {
        opts.has_square_override = true;
        opts.square_override = cliCfg.square;
    }

    PolygonGenerator gen;
    return gen.run(opts);
}
