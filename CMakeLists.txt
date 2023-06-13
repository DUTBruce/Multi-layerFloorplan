# CMakeList.txt: 顶层 CMake 项目文件，在此处执行全局配置
# 并包含子项目。
#
cmake_minimum_required (VERSION 3.8)
project ("floorplanning_iccad2023_btree")

set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3")

set(CMAKE_CXX_STANDARD 14)
add_executable(floorplanning_iccad2023_btree main.cpp Solver.cpp Config.hpp BStarTree.cpp Net.hpp Contour.hpp)
