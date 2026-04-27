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

## Phase 3: ROS 2 Humble with rules_ros (Comparison)
このフェーズでは、BazelのROS 2ルールである `rules_ros` を導入し、手動でのパス指定からルールベースの管理へ移行します。`mvukov/rules_ros2` と `ApexAI/rules_ros` の2つを比較検討します。

### 3.1. Common Strategy
- **Format**: 互換性と実績を考慮し、`WORKSPACE` 形式を維持します。
- **Approach**: 初回ビルドの成功と安定性を優先し、システムの `/opt/ros/humble` をインポートして利用する方式（Local ROS integration）をまず採用します。

### 3.2. mvukov/rules_ros2 Integration
- **Workspace**: `phase3/mvukov_rules_ros2/`
- **Setup**: `mvukov/rules_ros2` を `WORKSPACE` で設定し、ローカルのHumble環境をインポートします。
- **Package**: `src/humble_opt/` 配下の既存コードを `rules_ros2` のマクロ（`ros2_cpp_binary`等）でビルドできるように `BUILD` ファイルを書き換えます。

### 3.3. ApexAI/rules_ros Integration
- **Workspace**: `phase3/apexai_rules_ros/`
- **Setup**: `ApexAI/rules_ros` を `WORKSPACE` で設定。
- **Package**: 同様に既存コードの `BUILD` ファイルを書き換えてビルドを確認します。

## Verification Plan
- [ ] `mvukov/rules_ros2` 版のビルド成功と実行確認
- [ ] `ApexAI/rules_ros` 版のビルド成功と実行確認
- [ ] 両者の記述量、柔軟性、ビルド速度等の比較
