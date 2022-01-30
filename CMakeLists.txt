cmake_minimum_required(VERSION 3.21)
project(IK)
find_package(Eigen3 REQUIRED)
find_package(Pangolin 0.4 REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})
set(CMAKE_CXX_STANDARD 14)
include_directories(${EIGEN3_INCLUDE_DIR})
add_executable(IK main.cpp include/IK/IK.cpp include/IK/IK.h)
target_link_libraries(IK pango_display pango_plot)
target_include_directories(IK PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/IK>
        )
