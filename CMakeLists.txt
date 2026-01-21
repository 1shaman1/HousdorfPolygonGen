# CMakeList.txt: файл проекта CMake верхнего уровня; проведите глобальную конфигурацию
# и включите сюда подпроекты.
#
cmake_minimum_required(VERSION 3.10)

project(HousdorfPolygonGen LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(polygon_gen "HausdorffPolygonGen/HausdorffPolygonGen.cpp" "main.cpp" "HausdorffPolygonGen/HausdorffPolygon.h" "HausdorffPolygonGen/Point.h" "HausdorffPolygonGen/HausdorffPolygon.cpp")

if (MSVC)
    target_compile_options(polygon_gen PRIVATE /W4 /permissive-)
else()
    target_compile_options(polygon_gen PRIVATE -Wall -Wextra -Wpedantic)
endif()

