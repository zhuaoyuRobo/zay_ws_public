cmake_minimum_required(VERSION 3.5)
project(ndt_omp_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(PCL 1.12 REQUIRED)
find_package(OpenMP REQUIRED COMPONENTS CXX)

###########
## Build ##
###########
add_library(ndt_omp
  src/pclomp/voxel_grid_covariance_omp.cpp
  src/pclomp/ndt_omp.cpp
  src/pclomp/gicp_omp.cpp
)
target_include_directories(ndt_omp
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
    ${PCL_INCLUDE_DIRS}
)
target_link_libraries(ndt_omp
  PUBLIC
    ${PCL_LIBRARIES}
    OpenMP::OpenMP_CXX
)

add_executable(align
  apps/align.cpp
)
target_link_libraries(align
  ndt_omp
)
ament_target_dependencies(align
  rclcpp
)

#############
## Install ##
#############
install(
  TARGETS
    ndt_omp
  EXPORT export_ndt_omp
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    INCLUDES DESTINATION include
)
install(
  TARGETS align
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY "include/"
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_public_api test/test_public_api.cpp)
  target_link_libraries(test_public_api ndt_omp)
endif()

ament_export_targets(export_ndt_omp HAS_LIBRARY_TARGET)
ament_export_dependencies(PCL OpenMP)
ament_package()
