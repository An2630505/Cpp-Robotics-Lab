cmake_minimum_required(VERSION 3.14)
project(PNC-Robotics-Lab VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Eigen3
find_package(Eigen3 REQUIRED)

# pybind11
find_package(pybind11 REQUIRED)

add_subdirectory(pnc)
add_subdirectory(engine/physics)
