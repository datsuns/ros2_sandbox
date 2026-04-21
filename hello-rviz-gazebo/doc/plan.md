# ROS 2 学習カリキュラム：RViz & Gazebo 徹底マスター (Humble to Jazzy)

ROS 2の学習において、RViz（可視化）とGazebo（シミュレーション）の役割を明確に区別し、Humbleから最新のJazzyへの移行も見据えたカリキュラムです。

## 概要
本カリキュラムでは、「ロボットが見ている世界（RViz）」と「現実（仮想）の世界（Gazebo）」の違いを意識しながら、最終的に自作ロボットをシミュレーション環境で動かし、そのデータを可視化することを目指します。

## カリキュラム構成

### [フェーズ1：環境構築と基礎確認](file:///workspaces/sandbox/hello-rviz-gazebo/doc/phase1_env.md)
- **目標**: 開発環境を整え、ROS 2の基本ノード通信を理解する。
- **内容**:
    - devcontainerを利用した ROS 2 Humble環境の構築
    - GUI（RViz/Gazebo）を表示するための接続設定
    - Node, Topic, Messageの基礎復習（コマンドラインでの確認）

### [フェーズ2：RViz 2 深掘り - 「ロボットの頭の中を見る」](file:///workspaces/sandbox/hello-rviz-gazebo/doc/phase2_rviz.md)
- **目標**: RVizが「何を表示するためのツールか」を理解する。
- **内容**:
    - TF (Transform) の概念：座標系の管理と可視化
    - 各種Display項目の使い方（RobotModel, LaserScan, PointCloud2, Camera）
    - 静的なデータの可視化（ダミーデータを用いた演習）

### [フェーズ3：Gazebo 深掘り - 「仮想の物理世界を作る」](file:///workspaces/sandbox/hello-rviz-gazebo/doc/phase3_gazebo.md)
- **目標**: Gazeboが「物理的な相互作用をシミュレートする場所」であることを理解する。
- **内容**:
    - WorldファイルとModelファイルの構造
    - 物理エンジン（重力、摩擦、衝突）の設定
    - センサープラグインの追加（LiDARやカメラを仮想ロボットに搭載）

### [フェーズ4：連携の理解 - 「シミュレーションと可視化の融合」](file:///workspaces/sandbox/hello-rviz-gazebo/doc/phase4_bridge.md)
- **目標**: Gazebo上のロボット情報のRVizで表示する一連の流れを習得する。
- **内容**:
    - `ros_gz` (Gazebo Sim) と `gazebo_ros_pkgs` (Gazebo Classic) の違い
    - Gazebo内のセンサーデータをROS Topicとして配信し、RVizで表示
    - 「シミュレーション上の真値」と「ロボットが認識する観測値」の比較

### [フェーズ5：実践プロジェクトと最新動向 (Jazzy Jaliscoへの展望)](file:///workspaces/sandbox/hello-rviz-gazebo/doc/phase5_project.md)
- **目標**: ゼロからロボットモデルを作成し、最新バージョンでの変更点を理解する。
- **内容**:
    - URDF/Xacroを用いたロボットモデル作成（RViz用の見た目とGazebo用の物理特性）
    - 差動二輪ロボットの制御（Teleopを用いた遠隔操作）
    - **Jazzyへの移行点**: Gazebo Sim (Harmonic) への対応、Python 3.12環境、ミドルウェアの動向など

---

## 開発環境構成
- **OS**: Ubuntu 22.04 (Jammy)
- **ROS 2**: Humble Hawksbill
- **GUI**: WSLg (Windows 10/11 WSL2)
- **Gazebo**: Gazebo Sim (Fortress) をメインに使用

## 学習方針
1. **ROS 2のバージョン**: Humbleをベースに開始し、フェーズ5でJazzyでの変化点を学習。
2. **Gazeboの選択**: 今後の主流である Gazebo Sim をメインとしつつ、Classicとのアーキテクチャの違いもフェーズ4で学習。
