cmake_minimum_required(VERSION 3.10)
project(SafePlanner CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(include)
add_executable(safe_planner src/main.cpp src/LPAStarPlanner.cpp)