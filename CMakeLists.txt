# CMakeList.txt: файл проекта CMake верхнего уровня; проведите глобальную конфигурацию
# и включите сюда подпроекты.
#
cmake_minimum_required(VERSION 3.10)

project(HousdorfPolygonGen LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(polygon_gen
  src/HausdorffPolygonGen/HausdorffPolygonGen.cpp
  src/HausdorffPolygonGen/HausdorffPolygon.cpp
  src/main.cpp
)

target_include_directories(polygon_gen PRIVATE
  ${CMAKE_SOURCE_DIR}/src
  ${CMAKE_SOURCE_DIR}/src/HausdorffPolygonGen)

if (MSVC)
    target_compile_options(polygon_gen PRIVATE /W4 /permissive-)
else()
    target_compile_options(polygon_gen PRIVATE -Wall -Wextra -Wpedantic)
endif()
