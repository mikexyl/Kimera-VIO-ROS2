FROM cuda-ros-base:jazzy

ARG USERNAME=vscode
ARG USER_UID=1000
ENV ROS_DISTRO=jazzy
ARG PIXI_VERSION=v0.55.0

ENV DEBIAN_FRONTEND=noninteractive

# Install basic dependencies
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    ca-certificates \
    gpg \
    wget \
    software-properties-common \
    apt-transport-https

# install ros-perception dependencies
RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-perception \
    ros-$ROS_DISTRO-rviz2 \
    ros-$ROS_DISTRO-realsense2-camera \
    ros-$ROS_DISTRO-rqt-graph \
    ros-$ROS_DISTRO-rqt-tf-tree \
    ros-$ROS_DISTRO-image-pipeline \
    libspdlog-dev libgraphviz-dev kitty \
    libsuitesparse-dev \
    libmetis-dev libgflags-dev libgoogle-glog-dev googletest \
    libmkl-dev libblas-dev gdb && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# install taskfile
RUN sh -c "$(curl --location https://taskfile.dev/install.sh)" -- -d -b /usr/local/bin

# install develop tools
RUN apt-get update && apt-get install -y \
    python3-vcstool \
    python3-rosdep ccache ninja-build && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y curl bzip2 ca-certificates wget \
    && rm -rf /var/lib/apt/lists/*

# remove ubuntu user
RUN deluser --remove-home ubuntu || true

# create user
RUN groupadd --gid $USER_UID $USERNAME && \
    useradd --create-home --shell /bin/bash --uid $USER_UID --gid $USER_UID $USERNAME && \
    echo "$USERNAME ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers

RUN curl -L -o /usr/local/bin/pixi -fsSL --compressed "https://github.com/prefix-dev/pixi/releases/download/${PIXI_VERSION}/pixi-$(uname -m)-unknown-linux-musl" \
    && chmod +x /usr/local/bin/pixi \
    && pixi info

RUN apt update && apt install -y libunwind-dev libgoogle-glog-dev && apt-get clean && rm -rf /var/lib/apt/lists/*

# change to USER and install rust
USER $USERNAME
RUN curl https://sh.rustup.rs -sSf | sh -s -- -y && \
    echo 'source $HOME/.cargo/env' >> $HOME/.zshrc && \
    . "$HOME/.cargo/env" && \
    rustup default stable



