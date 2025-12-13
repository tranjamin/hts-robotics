# Start with an official ROS 2 base image for the desired 
# Do I need to fix this?
FROM ros:humble-ros-base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=humble

ARG FRANKA_PATH=franka_ros2

ARG USER_UID=1001
ARG USER_GID=1001
ARG USERNAME=user

# Install essential packages and ROS development tools
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        bash-completion \
        curl \
        gdb \
        git \
        nano \
        wget \
        openssh-client \
        python3-colcon-argcomplete \
        python3-colcon-common-extensions \
        sudo \
        vim \
        apt-utils \
        python3-pip \

        # for anygrasp and pyenv
        dialog \
        net-tools \
        build-essential \
        libncurses-dev \
        libreadline-dev \
        libbz2-dev \
        tk-dev \
        liblzma-dev \
        libffi-dev \
        libsqlite3-dev \

    && wget http://archive.ubuntu.com/ubuntu/pool/main/o/openssl/libssl1.1_1.1.1f-1ubuntu2_amd64.deb \
    && sudo dpkg -i libssl1.1_1.1.1f-1ubuntu2_amd64.deb \
    && rm libssl1.1_1.1.1f-1ubuntu2_amd64.deb \

    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Setup user configuration
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers \
    && echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/$USERNAME/.bashrc
    
USER $USERNAME

# Install some ROS 2 dependencies to create a cache layer
RUN sudo apt-get update \
    && sudo apt-get install -y --no-install-recommends \
        ros-humble-ros-gz \
        ros-humble-sdformat-urdf \
        ros-humble-joint-state-publisher-gui \
        ros-humble-ros2controlcli \
        ros-humble-controller-interface \
        ros-humble-hardware-interface-testing \
        ros-humble-ament-cmake-clang-format \
        ros-humble-ament-cmake-clang-tidy \
        ros-humble-controller-manager \
        ros-humble-ros2-control-test-assets \
        libignition-gazebo6-dev \
        libignition-plugin-dev \
        ros-humble-hardware-interface \
        ros-humble-control-msgs \
        ros-humble-backward-ros \
        ros-humble-generate-parameter-library \
        ros-humble-realtime-tools \
        ros-humble-joint-state-publisher \
        ros-humble-joint-state-broadcaster \
        ros-humble-moveit \
        ros-humble-moveit-setup-assistant \
        ros-humble-moveit-ros-move-group \
        ros-humble-moveit-kinematics \
        ros-humble-moveit-planners-ompl \
        ros-humble-moveit-ros-visualization \
        ros-humble-joint-trajectory-controller \
        ros-humble-moveit-simple-controller-manager \
        ros-humble-moveit-ros-perception \
        ros-humble-rviz2 \
        ros-humble-xacro \
        ros-humble-turtlesim \
        ros-humble-rqt \
        ros-humble-rqt-graph \
        ros-humble-realsense2-camera \
        ros-humble-realsense2-description \
    && sudo apt-get clean \
    && sudo rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws

# Install the missing ROS 2 dependencies
COPY . /ros2_ws/src
RUN sudo chown -R $USERNAME:$USERNAME /ros2_ws \
    && vcs import src < src/${FRANKA_PATH}/franka.repos --recursive --skip-existing \
    && sudo apt-get update \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y \
    && sudo apt-get clean \
    && sudo rm -rf /var/lib/apt/lists/* \
    && rm -rf /home/$USERNAME/.ros \
    && rm -rf src \
    && mkdir -p src

# Init pyenv
RUN curl -fsSL https://pyenv.run | bash
ENV PYENV_ROOT="/home/$USERNAME/.pyenv"
ENV PATH="$PYENV_ROOT/bin:$PYENV_ROOT/shims:${PATH}"
RUN echo 'eval "$(pyenv init --path)"' >> ~/.bashrc

# Install python 3.8
# RUN pyenv install 3.8
# RUN pyenv global 3.8

# Packages to install for Python 3.8
# RUN python -m pip install --upgrade pip
# RUN python -m pip install \
#     torch \
#     numpy==1.24.0 \
#     open3d

# RUN echo 'export PYENV_ROOT="$HOME/.pyenv"' >> ~/.bashrc && \
#     echo 'export PATH="$PYENV_ROOT/bin:$PYENV_ROOT/shims:$PATH"' >> ~/.bashrc && \
#     echo 'eval "$(pyenv init --path)"' >> ~/.bashrc && \
#     echo 'eval "$(pyenv init -)"' >> ~/.bashrc

COPY ./${FRANKA_PATH}/franka_entrypoint.sh /franka_entrypoint.sh
RUN sudo chmod +x /franka_entrypoint.sh

# Set the default shell to bash and the workdir to the source directory
SHELL [ "/bin/bash", "-c" ]
ENTRYPOINT [ "/franka_entrypoint.sh" ]
CMD [ "/bin/bash" ]
WORKDIR /ros2_ws