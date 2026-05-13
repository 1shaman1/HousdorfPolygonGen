#include "hausdorfgen_service.h"
#include "../HausdorffPolygonGen/HausdorffPolygonGen.h"
#include "../HausdorffPolygonGen/vertex_limits.h"
#include <grpcpp/grpcpp.h>
#include <hausdorf/v1/geometry.pb.h>
#include <spdlog/spdlog.h>
#include <cmath>

namespace hv1 = hausdorf::v1;

namespace {
    void fillProtoPolygon(hv1::Polygon* out, const std::vector<Point>& pts, bool is_convex) {
        out->set_is_convex(is_convex);
        out->set_vertex_count(static_cast<int32_t>(pts.size()));
        for (const auto& p : pts) {
            auto* pt = out->add_vertices();
            pt->set_x(static_cast<int32_t>(std::lround(p.x)));
            pt->set_y(static_cast<int32_t>(std::lround(p.y)));
        }
    }
}

grpc::Status HausdorfGenServiceImpl::GeneratePolygons(
    grpc::ServerContext*,
    const hausdorf::v1::GeneratePolygonsRequest* request,
    hausdorf::v1::GeneratePolygonsResponse* response) {
    const int minv = request->min_vertex_count();
    const int maxv = request->max_vertex_count();
    if (auto err = hausdorff_polygon_gen::validate_vertex_range(minv, maxv)) {
        return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, *err);
    }
    try {
        const int pc = request->polygon_count();
        PolygonGenerator gen;
        const auto polys = gen.generatePolygonsForRpc(minv, maxv, pc);
        for (const auto& poly : polys) {
            auto* pair = response->add_pairs();
            fillProtoPolygon(pair->mutable_convex_polygon(), poly.convexHull(), true);
            fillProtoPolygon(pair->mutable_non_convex_polygon(), poly.convex(), false);
        }
        return grpc::Status::OK;
    } catch (const std::invalid_argument& ex) {
        return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, ex.what());
    } catch (const std::exception& ex) {
        return grpc::Status(grpc::StatusCode::INTERNAL, ex.what());
    }
}

HausdorfGenServiceImpl::HausdorfGenServiceImpl() = default;

void run_grpc_server(const AppConfig& cfg) {
    std::string addr = "0.0.0.0:" + cfg.grpc_port;
    HausdorfGenServiceImpl service;
    grpc::ServerBuilder builder;
    builder.AddListeningPort(addr, grpc::InsecureServerCredentials());
    builder.RegisterService(&service);
    auto server = builder.BuildAndStart();
    spdlog::info("gRPC server listening on {}", addr);
    server->Wait();
}
