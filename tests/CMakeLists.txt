cmake_minimum_required(VERSION 3.14)
project(optimal_path_tracing_tests CXX)

include(CTest)
enable_testing()

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wshadow)

find_package(Catch2 3 REQUIRED)
add_executable(optimal_path_tracing_tests tests.cpp ../Node/package_opt/src/Delaunay/edge.cpp ../Node/package_opt/src/Delaunay/vector2.cpp)
target_link_libraries(optimal_path_tracing_tests PRIVATE Catch2::Catch2WithMain)

