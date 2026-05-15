#pragma once
#include "../config/config.h"
#include <hausdorf/v1/polygon_generator.grpc.pb.h>

class HausdorfGenServiceImpl final : public hausdorf::v1::PolygonGenerator::Service{
public:
    HausdorfGenServiceImpl();    

    grpc::Status GeneratePolygons(grpc::ServerContext* ctx,
         const hausdorf::v1::GeneratePolygonsRequest* request,
         hausdorf::v1::GeneratePolygonsResponse* response) override;

};

void run_grpc_server(const AppConfig& cfg);