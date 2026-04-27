.PHONY: all colcon-build bazel-build clean clean-colcon clean-bazel

all: colcon-build bazel-build

# colcon build
colcon-build:
	colcon build --base-paths src --packages-select pcl_sample diag_sample

# bazel build
bazel-build:
	bazel build //src/jazzy_opt/...

# Clean all build artifacts
clean: clean-colcon clean-bazel

clean-colcon:
	rm -rf build/ install/ log/

clean-bazel:
	bazel clean
