#include <iostream>
#include <string>
#include <vector>
#include <cstdlib>
#include <ctime>
#include "HausdorffPolygonGen/HausdorffPolygonGen.h"

void printHelp() {
    std::cout << "Polygon Generator\n\n";
    std::cout << "Usage:\n";
    std::cout << "  polygon_gen --max-vertices <N> [--count <K>] [--threads <T>] [--diff <D>] [--reflex <R>] [--depth-bisector <d>] [--depth-centroid <d>] [--out-dir <DIR>]\n\n";
    std::cout << "Required:\n";
    std::cout << "  --max-vertices <N>     Maximum number of vertices (>=6)\n";
    std::cout << "  At least one of:\n";
    std::cout << "    --diff <D>           Target area difference (convex hull area - polygon area)\n";
    std::cout << "    --reflex <R>         Target number of reflex (concave) angles\n";
    std::cout << "    --depth-bisector <d>     Target depth smooth on bisector (koefficient from neighboor edge length)\n\n";
    std::cout << "    --depth-centroid <d>     Target depth smooth to centroid (koefficient from  length)\n\n";
    std::cout << "Optional:\n";
    std::cout << "  --count <K>            Number of polygons to generate (default: 1)\n";
    std::cout << "  --threads <T>          Number of threads (default: hardware_concurrency)\n";
    std::cout << "  --out-dir <DIR>        Output folder name (default: auto-generated)\n";
    std::cout << "  --help, -h             Show this help message\n";
}

int main(int argc, char** argv) {
    double targetDiff = -1;
    int targetReflex = -1;
    //double targetAngle = -1;
    int maxVertices = -1;
    int count = 1;
    int threads = 0;
    double depthBisector = -1.0;
    double depthCentroid = -1.0;
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
        if (arg == "--max-vertices" && i + 1 < argc) {
            maxVertices = std::atoi(argv[++i]);
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
        else if (arg == "--diff" && i + 1 < argc) {
            targetDiff = std::atof(argv[++i]);
        }
        else if (arg == "--reflex" && i + 1 < argc) {
            targetReflex = std::atoi(argv[++i]);
        }
        /*else if (arg == "--max-angle" && i + 1 < argc) {
            targetAngle = std::atof(argv[++i]);
        }*/
        else if (arg == "--depth-bisector" && i + 1 < argc) {
            depthBisector = std::atof(argv[++i]);
        }
        else if (arg == "--depth-centroid" && i + 1 < argc) {
            depthCentroid = std::atof(argv[++i]);
        }
        else {
            std::cout << "Unknown argument: " << arg << "\n";
            return 1;
        }
    }

    if (maxVertices < 6) {
        std::cout << "Error: --max-vertices must be >= 6\n";
        return 1;
    }

    if (count < 1) {
        std::cout << "Error: --count must be >= 1\n";
        return 1;
    }

    if (targetDiff < 0 && targetReflex < 0 && depthBisector < 0 && depthCentroid < 0) {
        std::cout << "Error: At least one of --diff, --reflex, --max-angle must be specified\n";
        return 1;
    }

    if (depthBisector >= 0 && depthCentroid >= 0) {
        std::cout << "Error: --depth-bisector and --depth-centroid are mutually exclusive\n";
        return 1;
    }

    if (depthBisector < 0 && depthCentroid < 0) {
        std::cout << "Error: One of --depth-bisector or --depth-centroid must be specified\n";
        return 1;
    }

    if (threads <= 0) {
        threads = std::thread::hardware_concurrency();
        if (threads <= 0) threads = 2;
    }

    PolygonGenerator gen;

    if (depthBisector >= 0) {
        gen.reflexDir = ReflexDirection::Bisector;
        gen.depthCoeff = depthBisector;
    }
    else {
        gen.reflexDir = ReflexDirection::Centroid;
        gen.depthCoeff = depthCentroid;
    }

    bool ok = gen.generateMultipleParallel(
        targetDiff, targetReflex, maxVertices, count, outDir, threads
    );

    if (!ok) {
        std::cout << "NOT FOUND within limit\n";
        return 1;
    }

    std::cout << "Done.\n";
    return 0;
}
