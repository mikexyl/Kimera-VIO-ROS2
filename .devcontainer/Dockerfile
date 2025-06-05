FROM cuda-ros-base:humble

ARG USERNAME=vscode
ARG USER_UID=1000
ENV ROS_DISTRO=humble

# install ros-perception dependencies
RUN --mount=type=cache,target=/var/cache/apt \
    --mount=type=cache,target=/var/lib/apt \
    apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-perception \
    ros-$ROS_DISTRO-rviz2 \
    ros-$ROS_DISTRO-realsense2-camera \
    ros-$ROS_DISTRO-rqt-graph \
    libmetis-dev libgflags-dev libgoogle-glog-dev && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# install taskfile
RUN sh -c "$(curl --location https://taskfile.dev/install.sh)" -- -d -b /usr/local/bin

# install ros2 develop tools
RUN --mount=type=cache,target=/var/cache/apt \
    --mount=type=cache,target=/var/lib/apt \
    apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-rosdep ccache && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y curl bzip2 ca-certificates wget \
    && rm -rf /var/lib/apt/lists/*

# change to USER and install rust
USER $USERNAME
RUN curl https://sh.rustup.rs -sSf | sh -s -- -y && \
    echo 'source $HOME/.cargo/env' >> $HOME/.bashrc && \
    . "$HOME/.cargo/env" && \
    rustup default stable

# install Miniforge (Mamba/Conda) for the user
# RUN wget --quiet https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-x86_64.sh -O /tmp/miniforge.sh && \
#     bash /tmp/miniforge.sh -b -p $HOME/miniforge && \
#     rm /tmp/miniforge.sh && \
#     echo 'export PATH="$HOME/miniforge/bin:$PATH"' >> $HOME/.bashrc && \
#     $HOME/miniforge/bin/conda init bash



# switch back to root
USER root

