#include "config/config.h"
#include "grpc/hausdorfgen_service.h"

#include <spdlog/spdlog.h>

#include <iostream>
#include <string>
#include <thread>
#include <vector>

void printHelp() {
    std::cout << "Polygon Generator (Random)\n\n";
    std::cout << "Usage:\n";
    std::cout << "  polygon_gen [--count <K>] [--threads <T>] [--out-dir <DIR>]\n\n";
    std::cout << "Optional:\n";
    std::cout << "  --count <K>                Number of polygons to generate (default: 1)\n";
    std::cout << "  --threads <T>              Number of threads (default: hardware_concurrency)\n";
    std::cout << "  --out-dir <DIR>            Output folder name (default: auto-generated)\n";
    std::cout << "  --help, -h                 Show this help message\n";
}

int main(int argc, char** argv) {
    std::string config_path = "appConfig.cfg";
    if (argc > 1) config_path = argv[1];

    try{
        spdlog::info("Loading config from {}", config_path);
        auto cfg = load_config(config_path);
        std::thread grpc_thread([&] {
            run_grpc_server(cfg);
        });

        grpc_thread.join();

    } catch (const std::exception& ex) {
        spdlog::error("Fatal: {}", ex.what());
        return 1;
    }

    return 0;
}
