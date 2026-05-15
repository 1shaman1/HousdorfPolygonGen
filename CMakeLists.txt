cmake_minimum_required(VERSION 3.16)

if(DEFINED ENV{VCPKG_ROOT} AND NOT DEFINED CMAKE_TOOLCHAIN_FILE)
  file(TO_CMAKE_PATH "$ENV{VCPKG_ROOT}" _VCPKG_ROOT)
  set(CMAKE_TOOLCHAIN_FILE "${_VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake"
      CACHE STRING "Vcpkg toolchain file")
endif()

project(HousdorfPolygonGen LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(HAUSDORF_PROTO_ROOT "${CMAKE_SOURCE_DIR}/../contracts/proto" CACHE PATH
    "Directory that contains hausdorf/v1/*.proto")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/../orchestrator/cmake")
include(proto)

find_package(Protobuf CONFIG REQUIRED)
find_package(gRPC CONFIG REQUIRED)
find_package(spdlog CONFIG REQUIRED)
find_package(libconfig CONFIG REQUIRED)
# MSVC: libconfig::libconfig / libconfig::libconfig++. Unix upstream: libconfig::config / libconfig::config++.
if(TARGET libconfig::libconfig++)
  set(_hausdorf_libconfig_cxx libconfig::libconfig++)
elseif(TARGET libconfig::config++)
  set(_hausdorf_libconfig_cxx libconfig::config++)
else()
  message(FATAL_ERROR "libconfig C++ target not found (libconfig::libconfig++ or libconfig::config++)")
endif()
if(TARGET libconfig::libconfig)
  set(_hausdorf_libconfig_c libconfig::libconfig)
elseif(TARGET libconfig::config)
  set(_hausdorf_libconfig_c libconfig::config)
else()
  set(_hausdorf_libconfig_c "")
endif()

hausdorf_add_proto_library(hausdorf_contracts)

add_executable(polygon_gen
  src/HausdorffPolygonGen/HausdorffPolygonGen.cpp
  src/HausdorffPolygonGen/HausdorffPolygon.cpp
  src/config/config.cpp
  src/grpc/hausdorfgen_service.cpp
  src/main.cpp
)

target_include_directories(polygon_gen PRIVATE ${CMAKE_SOURCE_DIR}/src)

target_link_libraries(polygon_gen PRIVATE
  hausdorf_contracts
  spdlog::spdlog
  ${_hausdorf_libconfig_cxx}
)
if(_hausdorf_libconfig_c)
  target_link_libraries(polygon_gen PRIVATE ${_hausdorf_libconfig_c})
endif()

if(MSVC)
  target_compile_options(polygon_gen PRIVATE /W4 /permissive-)
else()
  target_compile_options(polygon_gen PRIVATE -Wall -Wextra -Wpedantic)
endif()
