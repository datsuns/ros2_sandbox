cmake_minimum_required(VERSION 3.8)
project(simple_unittest)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

add_library(simple_node src/simple_node.cpp)

target_include_directories(simple_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(simple_node rclcpp std_msgs)


install(TARGETS
  simple_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # コードカバレッジ用
  set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} --coverage") # 追加
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --coverage") # 追加


  # テスト用定義
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(${PROJECT_NAME}_test test/test_simple_node.cpp)

  target_include_directories(${PROJECT_NAME}_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  ament_target_dependencies(${PROJECT_NAME}_test
    std_msgs
  )
  target_link_libraries(${PROJECT_NAME}_test simple_node)
endif()


ament_package()
