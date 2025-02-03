default:
	colcon build

run:
	# source ./install/setup.bash
	ros2 run cpp_pubsub talker

create_package:
	mkdir -p src && cd src && \
	ros2 pkg create --build-type ament_cmake --license Apache-2.0 cpp_pubsub

.PHONY: default run create_package
