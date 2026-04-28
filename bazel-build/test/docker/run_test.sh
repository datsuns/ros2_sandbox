#!/bin/bash
set -e

export MSYS_NO_PATHCONV=1
BUILD_CONTAINER="angry_curran"
TEST_CONTAINER="ros2-test-container"
BUILD_ROOT="/workspaces/sandbox/bazel-build/phase3/mvukov_rules_ros2"

# スクリプトのディレクトリに移動
cd "$(dirname "$0")"

echo "=== ROS 2 Test Container Starter ==="

# 1. ビルドコンテナの特定
# 名前またはラベルに 'bazel-build' を含む実行中のコンテナを探します
echo "Searching for build container..."
# BUILD_CONTAINER=$(docker ps --filter "status=running" --format "{{.ID}} {{.Names}}" | grep "bazel-build" | awk '{print $1}' | head -n 1)

if [ -z "$BUILD_CONTAINER" ]; then
    echo "Error: 'bazel-build' を含むビルドコンテナが見つかりませんでした。"
    echo "ビルドコンテナ（devcontainer）が起動していることを確認してください。"
    exit 1
fi

echo "Found build container: $BUILD_CONTAINER"

# 2. Bazelバイナリの抽出
echo "Extracting Bazel binaries from build container..."
# 既存の bin ディレクトリがあれば削除
rm -rf ./bin
mkdir -p ./bin

# # ビルドコンテナ内のバイナリを検索してコピー
# # Bazelの出力先は /root/.cache 配下の複雑なパスにあるため、find で探します
# for target in "pcl_sample_node" "diag_sample_node"; do
#     echo "Searching for $target..."
#     # コンテナ内で find を実行
#     BINARY_PATH=$(docker exec "$BUILD_CONTAINER" find /home/rosuser/.cache -name "$target" -type f -executable | head -n 1)
#     
#     if [ -n "$BINARY_PATH" ]; then
#         echo "Found $target at: $BINARY_PATH"
#         docker cp "$BUILD_CONTAINER":"$BINARY_PATH" ./bin/
#     else
#         echo "Warning: $target could not be found in the build container."
#     fi
# done

docker exec $BUILD_CONTAINER tar -chf /tmp/bazel-bin.tar -C $BUILD_ROOT bazel-bin
docker cp $BUILD_CONTAINER:/tmp/bazel-bin.tar ./bin/

if [ -z "$(ls -A ./bin)" ]; then
    echo "Error: バイナリが一つも抽出できませんでした。"
    exit 1
fi

echo "Extraction successful."

# 3. テストコンテナのビルドと起動
echo "Building and starting test container (Ubuntu 22.04)..."
docker compose down
docker compose up -d

docker exec -w /test/ ros2-test-container tar -xvf bazel-bin.tar
# docker exec -w /test/bazel-bin/src/humble_opt/diag_sample/diag_sample_node.runfiles/_main ros2-test-container ../../diag_sample_node
# docker exec -w /test/bazel-bin/src/humble_opt/pcl_sample/pcl_sample_node.runfiles/_main ros2-test-container ../../pcl_sample_node
