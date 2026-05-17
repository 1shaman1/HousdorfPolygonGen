cmake_minimum_required(VERSION 3.10)

project(HousdorfPolygonGen LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(GEN_SOURCES
    main.cpp
    HausdorffPolygonGen/HausdorffPolygonGen.cpp
    HausdorffPolygonGen/HausdorffPolygon.cpp
    HausdorffPolygonGen/geometry.cpp
    HausdorffPolygonGen/square_sampler.cpp
    HausdorffPolygonGen/dent_builder.cpp
    HausdorffPolygonGen/edge_pocket_builder.cpp
    HausdorffPolygonGen/metric_calculator.cpp
    HausdorffPolygonGen/alpha_lebedev.cpp
    HausdorffPolygonGen/experiment_config.cpp
    HausdorffPolygonGen/sweep_planner.cpp
    HausdorffPolygonGen/polygon_io.cpp
    HausdorffPolygonGen/case_generator.cpp
    HausdorffPolygonGen/batch_runner.cpp
    HausdorffPolygonGen/hausdorff_enricher.cpp
)

add_executable(polygon_gen ${GEN_SOURCES})

target_include_directories(polygon_gen PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

if (MSVC)
    target_compile_options(polygon_gen PRIVATE /W4 /permissive-)
else()
    target_compile_options(polygon_gen PRIVATE -Wall -Wextra -Wpedantic)
endif()
