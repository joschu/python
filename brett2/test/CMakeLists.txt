cmake_minimum_required(VERSION 2.4.6)



set(CMAKE_BUILD_TYPE Debug)


find_package(Boost REQUIRED COMPONENTS python)
link_directories(
  /usr/local/lib
)

include_directories(
  /usr/include/python2.6
  /usr/lib/python2.6/dist-packages/numpy/core/include
)

add_executable(use_pr2_from_cpp use_pr2_from.cpp)
target_link_libraries(use_pr2_from_cpp boost_python python2.6)