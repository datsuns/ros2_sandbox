PKG_NAME := compo_multi

default:
	colcon build

run:
	#ros2 run $(PKG_NAME) listener
	ros2 launch $(PKG_NAME) launch.py

manu:
	ros2 component load /ComponentManager $(PKG_NAME) MinimalSubscriber

pub:
	ros2 run $(PKG_NAME) talker

loader:
	# ros2 run rclcpp_components component_container_mt --ros-args --log-level info -p thread_num:=4
	ros2 run rclcpp_components component_container_mt --ros-args --log-level info -p thread_num:=1

sub:
	ros2 run $(PKG_NAME) manu_listener --ros-args -p thread_num:=4


create_pkg:
	mkdir -p src && cd src && \
		ros2 pkg create --build-type ament_cmake --license Apache-2.0 $(PKG_NAME) && \
		cd $(PKG_NAME)/src && \
		wget -O publisher_member_function.cpp https://raw.githubusercontent.com/ros2/examples/humble/rclcpp/topics/minimal_publisher/member_function.cpp && \
		wget -O subscriber_member_function.cpp https://raw.githubusercontent.com/ros2/examples/humble/rclcpp/topics/minimal_subscriber/member_function.cpp

.PHONY: default run pub create_pkg

