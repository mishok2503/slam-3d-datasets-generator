cmake_minimum_required(VERSION 3.21)
project(MapGenerator)

set(CMAKE_CXX_STANDARD 20)

add_executable(MapGenerator main.cpp Cell.h Map.h MapGenerator.h CubeMapGenerator.h Simulator.cpp Simulator.h Robot.h)
