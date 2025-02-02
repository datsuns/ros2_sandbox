cmake_minimum_required(VERSION 3.8)
project(compo_multi)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

add_library(listener_component SHARED src/subscriber_member_function.cpp)
ament_target_dependencies(listener_component rclcpp std_msgs rclcpp_components)
rclcpp_components_register_node(
    listener_component
    PLUGIN "MinimalSubscriber"
    EXECUTABLE listener
)

add_executable(manu_listener src/subscriber.cpp)
ament_target_dependencies(manu_listener rclcpp std_msgs)

install(TARGETS
  talker
  manu_listener
  DESTINATION lib/${PROJECT_NAME})

ament_export_targets(export_listener_component)
install(TARGETS listener_component
        EXPORT export_listener_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
