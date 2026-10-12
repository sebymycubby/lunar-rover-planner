cmake_minimum_required(VERSION 3.14)
project(lunar_rover CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

add_library(rover_core src/map.cpp src/planner.cpp src/mission.cpp)
target_include_directories(rover_core PUBLIC include)
target_compile_options(rover_core PRIVATE -Wall -Wextra)

add_executable(lunar_rover src/main.cpp)
target_link_libraries(lunar_rover PRIVATE rover_core)
target_compile_options(lunar_rover PRIVATE -Wall -Wextra)

enable_testing()
add_executable(test_planner tests/test_planner.cpp)
target_link_libraries(test_planner PRIVATE rover_core)
add_test(NAME planner_tests COMMAND test_planner)
