#pragma once

#include <optional>
#include <string>

namespace hausdorff_polygon_gen {

constexpr int kMinVertexCount = 5;
constexpr int kMaxVertexCount = 30;

/// @return Сообщение об ошибке, если диапазон недопустим; иначе std::nullopt.
inline std::optional<std::string> validate_vertex_range(int min_vertex_count,
                                                       int max_vertex_count) {
    if (min_vertex_count < kMinVertexCount || min_vertex_count > kMaxVertexCount) {
        return "min_vertex_count must be between " + std::to_string(kMinVertexCount) + " and " +
               std::to_string(kMaxVertexCount) + " inclusive";
    }
    if (max_vertex_count < kMinVertexCount || max_vertex_count > kMaxVertexCount) {
        return "max_vertex_count must be between " + std::to_string(kMinVertexCount) + " and " +
               std::to_string(kMaxVertexCount) + " inclusive";
    }
    if (min_vertex_count > max_vertex_count) {
        return "min_vertex_count must not exceed max_vertex_count";
    }
    return std::nullopt;
}

}  // namespace hausdorff_polygon_gen
