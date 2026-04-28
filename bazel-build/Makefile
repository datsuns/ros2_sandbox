.PHONY: all colcon-build colcon-build-humble colcon-run-pcl-humble colcon-run-diag-humble colcon-run-pcl-jazzy colcon-run-diag-jazzy bazel-build bazel-build-humble clean clean-colcon clean-bazel

all: colcon-build colcon-build-humble bazel-build bazel-build-humble

# colcon build
colcon-build:
	colcon build --base-paths src --packages-select pcl_sample diag_sample

colcon-build-humble:
	colcon build --base-paths src --packages-select pcl_sample_humble diag_sample_humble

# colcon run
colcon-run-pcl-humble:
	. install/setup.bash && ros2 run pcl_sample_humble pcl_sample_node

colcon-run-diag-humble:
	. install/setup.bash && ros2 run diag_sample_humble diag_sample_node

colcon-run-pcl-jazzy:
	. install/setup.bash && ros2 run pcl_sample pcl_sample_node

colcon-run-diag-jazzy:
	. install/setup.bash && ros2 run diag_sample diag_sample_node

# bazel build
bazel-build:
	bazel build //src/jazzy_opt/...

bazel-build-humble:
	bazel build //src/humble_opt/...

bazel-run-diag:
	bazel run --subcommands //src/humble_opt/diag_sample:diag_sample_node

bazel-run-pcl:
	bazel run --subcommands //src/humble_opt/pcl_sample:pcl_sample_node

# Clean all build artifacts
clean: clean-colcon clean-bazel

clean-colcon:
	rm -rf build/ install/ log/

clean-bazel:
	bazel clean
