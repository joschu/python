cmake_minimum_required(VERSION 2.8)
project(fastmath)
set(CMAKE_BUILD_TYPE Release)
find_package(Eigen REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR})
include(${CMAKE_SOURCE_DIR}/boost-python.cmake)

include_directories(${EIGEN_INCLUDE_DIRS})
boost_python_module(fastmath fastmath.cpp)