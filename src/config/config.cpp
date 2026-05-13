#include "config.h"
#include <libconfig.h++>
#include <stdexcept>

namespace {
    std::string read_required_string(const libconfig::Setting& root, const char* key) {
        if (!root.exists(key)) {
            throw std::runtime_error(std::string("Key '") + key + "' not found in config");
        }
        return static_cast<const char*>(root[key]);
    }
}

AppConfig load_config(const std::string& config_path) {
    libconfig::Config cfg;
    cfg.readFile(config_path.c_str());
    const auto& root = cfg.getRoot();
    AppConfig config;
    config.grpc_port = read_required_string(root, "grpc_port");
    config.orchestrator_addr = read_required_string(root, "orchestrator_addr");
    return config;
}