cmake_minimum_required(VERSION 3.5)
project(ndt_omp_ros2_consumer)

find_package(ndt_omp_ros2 REQUIRED)

add_executable(consumer main.cpp)
target_link_libraries(consumer ndt_omp_ros2::ndt_omp)
