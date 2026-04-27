# Phase 3: ROS 2 rules_ros Comparison - Current Status

## 概要
ROS 2 Humble環境において、`mvukov/rules_ros2` と `ApexAI/rules_ros` の2つのBazelルールをソースビルド方式で検証中。
現在、コンテナが `root` ユーザーで実行されていることに起因する Bazel の制限、および特定のコンパイルエラーに直面している。

---

## 1. mvukov/rules_ros2 (phase3/mvukov_rules_ros2)
- **進捗**: `diag_sample` のビルドに成功。
- **解決した課題**: 
  - `rclcpp` のコンパイルエラーは、ビルドオプションに `-std=c++17` を明示的に追加することで解消。
  - 非rootユーザー (`rosuser`) での実行により、Bazel の権限問題も解決。
- **残っている課題**: 
  - `pcl_sample` が依存する `ros2_pcl_conversions` が `rules_ros2` のデフォルトリポジトリリストに含まれていない。
  - `rules_ros2` はソースビルド方式をとっているため、未対応のパッケージを利用するには個別の `BUILD` ファイル定義が必要。

## 2. ApexAI/rules_ros (phase3/apexai_rules_ros)
- **進捗**: 依存関係のフェッチまで到達したが、パッケージ不足で停止。
- **解決した課題**: 非rootユーザーでの実行により `rules_python` の制限を回避。
- **直面している課題**:
  - `ApexAI/rules_ros` は多くの ROS 2 パッケージが未だ「bazelized」されておらず、`diagnostic_updater` や `pcl_conversions` がデフォルトで利用できない。
  - リポジトリ構成が非常に厳格で、独自の Python インタープリタ設定等を要求する。
- **考察**: 現時点では `mvukov/rules_ros2` の方が対応パッケージ数が多く、実用的である。

---

## 次のステップ
1. **mvukov版での pcl_sample ビルド再挑戦**:
   - `pcl_conversions` のソースを `http_archive` で取得し、適切な `BUILD` ファイルを割り当てる設定を試みる。
2. **比較レポートの作成**:
   - Phase 1 (Local integration) と Phase 3 (Source build) の記述量、ビルド時間、ポータビリティの観点での比較。
   - `mvukov` と `ApexAI` の現状の完成度の差を記録。

---

## 作業ディレクトリ構造
- `/workspaces/sandbox/bazel-build/phase3/mvukov_rules_ros2/`
- `/workspaces/sandbox/bazel-build/phase3/apexai_rules_ros/`
