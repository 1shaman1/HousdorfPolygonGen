#include <iostream>
#include <string>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <thread>
#include "HausdorffPolygonGen/HausdorffPolygonGen.h"

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
    int count = 1;
    int threads = 0;
    std::string outDir = "";

    if (argc == 1) {
        printHelp();
        return 0;
    }

    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];

        if (arg == "--help" || arg == "-h") {
            printHelp();
            return 0;
        }
        else if (arg == "--count" && i + 1 < argc) {
            count = std::atoi(argv[++i]);
        }
        else if (arg == "--threads" && i + 1 < argc) {
            threads = std::atoi(argv[++i]);
        }
        else if (arg == "--out-dir" && i + 1 < argc) {
            outDir = argv[++i];
        }
        else {
            std::cout << "Unknown argument: " << arg << "\n";
            return 1;
        }
    }

    if (count < 1) {
        std::cout << "Error: --count must be >= 1\n";
        return 1;
    }

    if (threads <= 0) {
        threads = std::thread::hardware_concurrency();
        if (threads <= 0) threads = 2;
    }

    PolygonGenerator gen;

    gen.generateMultipleParallelRandom(count, outDir, threads);

    std::cout << "Done.\n";
    return 0;
}
