cmake_minimum_required(VERSION 3.14)
project(lunar_rover CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

include_directories(include)

add_executable(lunar_rover
    src/main.cpp
    src/map.cpp
    src/planner.cpp
)
