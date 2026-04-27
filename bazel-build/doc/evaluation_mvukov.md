# ROS 2 Bazel Build Evaluation: mvukov/rules_ros2

## 評価結果: 採用推奨 (High Priority)

現時点のプロジェクト要件（ROS 2 Humble, C++ノード, 標準パッケージ依存）において、最も実用的で安定したビルドを提供できる。

### 1. ビルドの成功状況
- **diag_sample_node**: 成功 (Build Success)
- **pcl_sample_node**: 一部成功 (ROS 2 パッケージの個別定義が必要)

### 2. 技術的特徴
- **Bzlmod (MODULE.bazel) 対応**: 現代的な Bazel のモジュール管理に対応しており、依存関係の記述がシンプル。
- **ソースビルド方式**: ROS 2 の各コンポーネントをソースからビルドするため、環境に依存しないビルドが可能。
- **柔軟性**: 必要な ROS 2 パッケージを選択してフェッチする機能がある。

### 3. 直面した課題と解決策
- **C++17 互換性**: 
  - `rclcpp` の一部のコード（CTADや `std::atomic`）がデフォルトのビルド設定ではエラーになる。
  - **解決策**: `.bazelrc` に `build --cxxopt=-std=c++17` を追加することで完全に解消。
- **所有権問題**: 
  - 前回のビルド（root実行）によるファイル所有権が原因で、一般ユーザーでのクリーンビルドが失敗した。
  - **解決策**: ワークスペースの所有権を `chown` で修正し、非rootユーザー (`rosuser`) で実行することで解決。

### 4. 制限事項
- `pcl_conversions` などの比較的一般的なパッケージでも、`rules_ros2` のデフォルト設定に含まれていない場合、自分で `MODULE.bazel` または `WORKSPACE` でリポジトリ定義を追加する必要がある。

### 5. 総合評価
ROS 2 パッケージの対応数が多く、コミュニティの活発さも感じられる。実プロジェクトへの導入において第一候補となる。

---

## 付録: 手動でのパッケージ追加手順 (pcl_conversions の例)

`mvukov/rules_ros2` の標準リポジトリリストに含まれていないパッケージを追加する場合のワークフロー。

### 1. リポジトリの定義 (WORKSPACE)
`http_archive` を使用して GitHub 等からソースを取得する。

```python
http_archive(
    name = "ros2_pcl_conversions",
    build_file = "//repositories:pcl_conversions.BUILD.bazel", # 外部ビルドファイルを指定
    strip_prefix = "perception_pcl-2.4.5",
    urls = ["https://github.com/ros-perception/perception_pcl/archive/refs/tags/2.4.5.tar.gz"],
)
```

### 2. カスタム BUILD ファイルの作成
取得したソースに対して、Bazel のルールを適用する `BUILD` ファイルを作成する。
- **C++ ライブラリの場合**: `cc_library` を使用。
- **メッセージパッケージの場合**: `ros2_interface_library` ルール（`@com_github_mvukov_rules_ros2//ros2:interfaces.bzl`）を使用して IDL からコード生成を行う。

### 3. MODULE.bazel での依存関係の公開
`non_module_deps` を通じて、そのパッケージが依存する他の ROS 2 パッケージを `use_repo` でメインレポジトリから見えるようにする。

```python
ros2 = use_extension("@com_github_mvukov_rules_ros2//ros2:extensions.bzl", "non_module_deps")
use_repo(
    ros2,
    "ros2_rclcpp",
    "ros2_common_interfaces", # 依存する標準パッケージを追加
)
```

### 4. 依存パッケージの解決
推移的な依存関係（例: `pcl_conversions` -> `pcl_msgs` -> `sensor_msgs`）を一つずつ解決していく必要がある。標準で用意されていないメッセージパッケージも同様に `http_archive` で追加可能。
