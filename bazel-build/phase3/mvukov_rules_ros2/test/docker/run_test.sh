#!/bin/bash
set -e

# スクリプトのディレクトリに移動
cd "$(dirname "$0")"

# 固定値の設定
BUILD_CONTAINER="angry_curran"
WORKSPACE_ROOT="/workspaces/sandbox/bazel-build/phase3/mvukov_rules_ros2"

echo "=== ROS 2 Test Container Starter (Phase 3 / rules_ros2) ==="

# 1. 抽出準備
echo "Preparing 'bin' directory..."
rm -rf ./bin
mkdir -p ./bin

# 2. バイナリと .runfiles の抽出
# rules_ros2 のバイナリは依存ライブラリを .runfiles に保持しているため、両方コピーします
for target in "pcl_sample_node" "diag_sample_node"; do
    echo "Processing $target..."
    
    # パス候補（pcl_sample か diag_sample か）
    if [ "$target" == "pcl_sample_node" ]; then
        pkg_dir="pcl_sample"
    else
        pkg_dir="diag_sample"
    fi
    
    SRC_BASE="${WORKSPACE_ROOT}/bazel-bin/src/humble_opt/${pkg_dir}/${target}"
    
    # バイナリ本体のコピー
    echo "Copying binary: $target"
    docker cp "${BUILD_CONTAINER}:${SRC_BASE}" ./bin/
    
    # runfiles ディレクトリのコピー
    echo "Copying runfiles: ${target}.runfiles"
    docker cp "${BUILD_CONTAINER}:${SRC_BASE}.runfiles" ./bin/
done

echo "Extraction successful."

# 3. テストコンテナのビルドと起動
echo "Building and starting test container..."
docker compose up --build -d

echo "=== Deployment Complete ==="
echo "You can attach to the container using: docker exec -it ros2-test-container bash"
echo "Binary location inside container: /test/bin/pcl_sample_node"
