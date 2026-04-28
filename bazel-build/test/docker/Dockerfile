FROM ubuntu:22.04

# 最小限のツールのインストール
RUN apt-get update && apt-get install -y \
    binutils

# 依存ライブラリのインストール
RUN apt-get install -y "libpcl-*1.12"

# 非rootユーザーの作成
ARG USERNAME=testuser
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME

# ワークスペースの設定
WORKDIR /test
RUN chown -R $USERNAME:$USER_GID /test

# # Bazelでビルドされたバイナリをコピー
# # 注意: run_test.sh によって事前にコピーされている必要があります
# COPY --chown=$USERNAME:$USER_GID bin/ /test/bin/
# 
# # 実行権限の付与
# RUN chmod +x /test/bin/*

# ユーザーの切り替え
USER $USERNAME

# デフォルトコマンド
CMD ["bash"]
