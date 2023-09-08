cmake_minimum_required(VERSION 3.21)
project(SLAM3dMapGenerator)

set(CMAKE_CXX_STANDARD 20)

add_executable(generator main.cpp Simulator.cpp)

target_include_directories(generator PRIVATE include)
target_include_directories(generator PRIVATE .)
