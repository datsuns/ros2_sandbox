cmake_minimum_required(VERSION 3.8)
project(pcl_sample)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pcl_conversions REQUIRED)

add_executable(pcl_sample_node src/pcl_sample_node.cpp)
target_include_directories(pcl_sample_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(pcl_sample_node PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(
  pcl_sample_node
  "rclcpp"
  "pcl_conversions"
)

install(TARGETS pcl_sample_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
