# Bazel WORKSPACE 解説 (Phase 1: ROS 2 Jazzy 直接参照)

このドキュメントでは、Phase 1 で構築した Bazel の `WORKSPACE` ファイルの内容とその意図について解説します。

## 概要

Phase 1 では、`rules_ros` などの外部 Bazel ルールを使用せず、システムにインストール済みの ROS 2 Jazzy (`/opt/ros/jazzy`) を Bazel から直接参照する設定を行いました。

## 設定の詳細

### 1. `new_local_repository` による外部リポジトリ定義

システム上のパスを Bazel の外部リポジトリとして扱うために `new_local_repository` を使用しています。

```python
new_local_repository(
    name = "ros2",
    path = "/opt/ros/jazzy",
    build_file_content = """...""",
)
```

- **`name`**: Bazel 内から `@ros2//...` として参照可能になります。
- **`path`**: システム上の参照先パスです。
- **`build_file_content`**: そのディレクトリに仮想的に作成される `BUILD` ファイルの内容です。

### 2. `cc_library` によるライブラリの定義

ROS 2 は多数のパッケージで構成されており、それぞれが異なるインクルードパスを必要とします。

#### `hdrs` (ヘッダーファイル)
`glob(["include/**"])` により、`/opt/ros/jazzy/include` 以下のすべてのヘッダーを対象としています。

#### `includes` (インクルードパスの設定)
ここが最も重要なポイントです。ROS 2 のパッケージは、ヘッダーの配置に以下のような不整合（仕様の差異）があります。

- **多くのパッケージ (例: `rclcpp`)**: `include/rclcpp/rclcpp/rclcpp.hpp` のようにネストされています。
  - コード内で `#include <rclcpp/rclcpp.hpp>` と書くためには、`include/rclcpp` をインクルードパスに追加する必要があります。
- **一部のパッケージ (例: `diagnostic_updater`)**: `include/diagnostic_updater/diagnostic_updater.hpp` のようにネストされていません。
  - コード内で `#include <diagnostic_updater/diagnostic_updater.hpp>` と書くためには、`include` をインクルードパスに追加する必要があります。

このため、`includes` リストには `include` ベースディレクトリと、各パッケージごとのサブディレクトリを大量に列挙しています。

#### `srcs` (共有ライブラリ)
`glob(["lib/*.so"])` により、`/opt/ros/jazzy/lib` 以下のすべての共有ライブラリをリンク対象としています。

### 3. PCL と Eigen の統合

ROS 2 パッケージ以外に依存しているシステムライブラリも同様の手法で統合しています。

- **`@pcl`**: `/usr/include/pcl-1.14` および `/usr/lib/x86_64-linux-gnu/libpcl_*.so` を参照。
- **`@eigen`**: `/usr/include/eigen3` を参照。

## この手法のメリットとデメリット

### メリット
- システム環境をそのまま利用できるため、ビルド環境の構築が（設定さえ書ければ）速い。
- 依存関係の構造（どのヘッダーがどこにあるか）を明示的に理解できる。

### デメリット
- **保守性が低い**: 新しい ROS 2 パッケージに依存するたびに `WORKSPACE` の `includes` リストを更新する必要がある。
- **環境依存**: `/opt/ros/jazzy` が存在しない環境ではビルドできない。
- **Bazel 的ではない**: Bazel は本来、ビルドに必要なすべてのソースを把握し、サンドボックス内で完結させるべきですが、この手法はシステムの絶対パスに大きく依存しています。

これらの課題を解決するために、Phase 3 では `rules_ros` を使用した、より Bazel らしい依存関係管理へ移行します。
