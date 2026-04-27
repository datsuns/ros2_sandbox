FROM ubuntu:22.04

# 最小限のツールのインストール
RUN apt-get update && apt-get install -y \
    binutils \
    && rm -rf /var/lib/apt/lists/*

# 非rootユーザーの作成
ARG USERNAME=testuser
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME

# ワークスペースの設定
WORKDIR /test
RUN chown -R $USERNAME:$USER_GID /test

# Bazelでビルドされたバイナリと.runfilesディレクトリをコピー
COPY --chown=$USERNAME:$USER_GID bin/ /test/bin/

# 実行権限の付与
RUN chmod +x /test/bin/*

# ユーザーの切り替え
USER $USERNAME

# デフォルトコマンド
CMD ["bash"]
