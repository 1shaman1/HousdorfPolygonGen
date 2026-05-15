#pragma once

#include <string>

struct AppConfig {
    std::string grpc_port;
    std::string orchestrator_addr;
};

AppConfig load_config(const std::string& config_path);
