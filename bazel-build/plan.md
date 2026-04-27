# ROS 2 Node Bazel Build Sample Plan

## Goal
Create a sample environment that demonstrates how to build ROS 2 nodes using both standard `colcon` and `bazel`. The sample will include packages that depend on `pcl_conversions` and `diagnostic_updater`. 

### Build Requirements
- **Bazel version**: 7.6.1
- **Bazel dependency management**: WORKSPACE-based

The Bazel build approach will be implemented in three phases, using separate packages for each step:
1. Use pre-installed packages in `/opt/ros` (Jazzy).
2. Change ROS 2 version to `humble`.
3. Use `rules_ros` with the Humble version.

## Phase 1: ROS 2 Jazzy with `/opt/ros`
### 1.1. Devcontainer Setup (Jazzy)
- Base image: `ros:jazzy`
- Install dependencies: `colcon`, `bazel`, `ros-jazzy-pcl-conversions`, `ros-jazzy-diagnostic-updater`.

### 1.2. Baseline colcon build (Jazzy)
- Create `src/jazzy_opt/pcl_sample` and `src/jazzy_opt/diag_sample`.
- Build and verify with `colcon build`.

### 1.3. Bazel build using `/opt/ros/jazzy`
- Initialize Bazel workspace.
- Create `BUILD` files in `src/jazzy_opt/pcl_sample` and `src/jazzy_opt/diag_sample`.
- Define local repository/targets to link `/opt/ros/jazzy` headers and libraries.
- Build and verify with `bazel build //src/jazzy_opt/...`.

## Phase 2: Change to ROS 2 Humble
### 2.1. Devcontainer Update (Humble)
- Modify `Dockerfile` and `devcontainer.json` to use `ros:humble` base image.
- Change dependencies to `ros-humble-pcl-conversions`, `ros-humble-diagnostic-updater`.
- Rebuild and reopen devcontainer.

### 2.2. Bazel build using `/opt/ros/humble`
- Create `src/humble_opt/pcl_sample` and `src/humble_opt/diag_sample`.
- Create `BUILD` files linking to `/opt/ros/humble`.
- Build and verify with `bazel build //src/humble_opt/...`.

## Phase 3: ROS 2 Humble with `rules_ros`
### 3.1. Bazel WORKSPACE Update
- Add `rules_ros` repository and dependencies to `WORKSPACE`.

### 3.2. Bazel build using `rules_ros`
- Create `src/humble_rules_ros/pcl_sample` and `src/humble_rules_ros/diag_sample`.
- Create `BUILD` files using `rules_ros` targets (e.g., `@ros2//:rclcpp`, `@ros2//:pcl_conversions` depending on rules_ros setup).
- Build and verify with `bazel build //src/humble_rules_ros/...`.

## Verification Plan
- Ensure each phase's devcontainer launches successfully.
- Ensure `colcon build` works for the baseline.
- Ensure `bazel build` works for each specific package in its respective phase.
- Run the generated binaries to ensure they execute properly.
