cmake_minimum_required(VERSION 3.14)
project(stanley CXX)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -pedantic)

add_executable(stanley main.cpp)