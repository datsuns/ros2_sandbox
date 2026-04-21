PKG_NAME := observe_env

default:
	colcon build

run:
	ros2 run $(PKG_NAME) $(PKG_NAME) --ros-args -p thread_num:=4

pub:
	ros2 run $(PKG_NAME) talker



create_pkg:
	mkdir -p src && cd src && \
		ros2 pkg create --build-type ament_cmake --license Apache-2.0 $(PKG_NAME) && \
		cd $(PKG_NAME)/src && \
		wget -O publisher_member_function.cpp https://raw.githubusercontent.com/ros2/examples/humble/rclcpp/topics/minimal_publisher/member_function.cpp 

.PHONY: default run pub create_pkg

