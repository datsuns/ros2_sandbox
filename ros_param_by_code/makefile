default:
	colcon build --packages-select cpp_parameters

run:
	# source ./install/setup.bash
	ros2 run cpp_parameters minimal_param_node

setup:
	rosdep install -i --from-path src --rosdistro humble -y

create_package:
	mkdir -p src && cd src && \
	ros2 pkg create --build-type ament_cmake --license Apache-2.0 cpp_parameters --dependencies rclcpp

.PHONY: default run setup create_package
