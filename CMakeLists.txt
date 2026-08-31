cmake_minimum_required(VERSION 3.10)
project(SafePlanner CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(include)

# The general-purpose planner: reads any graph from a file or stdin.
add_executable(safe_planner src/main.cpp src/ProblemLoader.cpp src/LPAStarPlanner.cpp)

# Regression test: the six hardcoded graphs from the assignment PDF.
add_executable(demo_test src/demo_test.cpp src/LPAStarPlanner.cpp)

# Automated benchmarking suite for experimental evaluation.
add_executable(benchmark src/benchmark.cpp src/LPAStarPlanner.cpp)

# Memory diagnostics (MemoryUtil.h) call GetProcessMemoryInfo on
# Windows, which lives in psapi.lib - link it there only. Linux/Mac need
# nothing extra (they read /proc/self/status directly).
if(WIN32)
    target_link_libraries(safe_planner psapi)
    target_link_libraries(benchmark psapi)
endif()