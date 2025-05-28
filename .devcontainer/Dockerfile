FROM cuda-ros-base:humble

ARG USERNAME=vscode
ARG USER_UID=1000
ENV ROS_DISTRO=humble

# create user and remove sudo password
RUN groupadd -g $USER_UID $USERNAME && \
    useradd -m -u $USER_UID -g $USERNAME -s /bin/bash $USERNAME && \
    echo "$USERNAME ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME
  
# install ros-perception dependencies
RUN --mount=type=cache,target=/var/cache/apt \
    --mount=type=cache,target=/var/lib/apt \
    apt-get update && apt-get install -y \
      ros-$ROS_DISTRO-perception \
      ros-$ROS_DISTRO-rviz2 \
      ros-$ROS_DISTRO-realsense2-camera \
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

# install micromamba
RUN apt-get update && apt-get install -y curl bzip2 ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# Set environment variables
ENV MAMBA_ROOT_PREFIX=/opt/micromamba
ENV PATH=$MAMBA_ROOT_PREFIX/bin:$PATH

# Install micromamba
RUN curl -Ls https://micro.mamba.pm/api/micromamba/linux-64/latest | tar -xvj -C /usr/local/bin/ --strip-components=1 bin/micromamba

# change to USER and install rust
USER $USERNAME
RUN curl https://sh.rustup.rs -sSf | sh -s -- -y && \
    echo 'source $HOME/.cargo/env' >> $HOME/.bashrc && \
    . "$HOME/.cargo/env" && \
    rustup default stable

# switch back to root
USER root

