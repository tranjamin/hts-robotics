FROM ros:humble-ros-base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=humble \
    CUDA_HOME=/usr/local/cuda-11.8 \
    PATH=/usr/local/cuda-11.8/bin:$PATH \
    LD_LIBRARY_PATH=/usr/local/cuda-11.8/lib64:$LD_LIBRARY_PATH \
    SKLEARN_ALLOW_DEPRECATED_SKLEARN_PACKAGE_INSTALL=True \
    CC=gcc-11 \
    CXX=g++-11 \
    TORCH_CUDA_ARCH_LIST="8.6;8.0;7.5;7.0;6.1;6.0;5.2;5.0" \
    FORCE_CUDA=1

ARG FRANKA_PATH=franka_ros2

ARG USER_UID=1001
ARG USER_GID=1001
ARG USERNAME=user

# Install essential packages and ROS development tools
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        apt-utils \
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
        dialog \
        net-tools \
        build-essential \
        libopenblas-dev \
        gcc-11 \
        g++-11 \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Install some ROS 2 dependencies to create a cache layer
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
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
        python3-rosdep \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Install CUDA toolkit 11.8
RUN wget -O /tmp/cuda-ubuntu2204.pin https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-ubuntu2204.pin \
    && mv /tmp/cuda-ubuntu2204.pin /etc/apt/preferences.d/cuda-repository-pin-600 \
    && wget -O /tmp/cuda-repo.deb https://developer.download.nvidia.com/compute/cuda/11.8.0/local_installers/cuda-repo-ubuntu2204-11-8-local_11.8.0-520.61.05-1_amd64.deb \
    && dpkg -i /tmp/cuda-repo.deb \
    && cp /var/cuda-repo-ubuntu2204-11-8-local/cuda-*-keyring.gpg /usr/share/keyrings/ \
    && apt-get update \
    && apt-get install -y --no-install-recommends cuda-toolkit-11-8 \
    && rm -f /tmp/cuda-repo.deb \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/* \
    && nvcc --version

# Install old libssl for license checking
RUN wget http://archive.ubuntu.com/ubuntu/pool/main/o/openssl/libssl1.1_1.1.1f-1ubuntu2_amd64.deb \
    && dpkg -i libssl1.1_1.1.1f-1ubuntu2_amd64.deb \
    && rm libssl1.1_1.1.1f-1ubuntu2_amd64.deb

# Setup user configuration
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers \
    && echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/$USERNAME/.bashrc
    
USER $USERNAME

# Install PyTorch with CUDA 11.8 both locally and in root
RUN sudo python3 -m pip install torch torchvision --index-url https://download.pytorch.org/whl/cu118
RUN python3 -m pip install torch torchvision --index-url https://download.pytorch.org/whl/cu118

ENV PATH=/home/$USERNAME/.local/bin:$PATH
ENV PYTHONPATH=/home/$USERNAME/.local/lib/python3.10/site-packages:$PYTHONPATH

# Copy MinkowskiEngine source
RUN pip install -U git+https://github.com/NVIDIA/MinkowskiEngine --no-deps

# Build GraspNetAPI
WORKDIR /build/GraspnetAPI
COPY anygrasp_sdk/dependencies/graspnetAPI .
RUN python3 -m pip install numpy==1.23.4 opencv-python scikit-image scipy open3d tqdm Pillow autolab_core autolab-perception cvxopt dill grasp_nms h5py pywavefront sklearn transforms3d==0.3.1 trimesh
RUN sudo -E python3 setup.py install --user

# Build PointNet2
WORKDIR /build/pointnet2
COPY anygrasp_sdk/pointnet2 .
RUN sudo python3 setup.py install

WORKDIR /
RUN sudo rm -rf /build

WORKDIR /ros2_ws

# Install the missing ROS 2 dependencies
COPY . /ros2_ws/src

# Install ROS dependencies via rosdep
RUN sudo chown -R $USERNAME:$USERNAME /ros2_ws \
    && vcs import src/franka_ros2 < src/dependencies.repos --recursive --skip-existing \
    && sudo apt-get update \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y \
    && sudo apt-get clean \
    && sudo rm -rf /var/lib/apt/lists/*

# # Check license registration
# RUN ls src/anygrasp_sdk/license_registration
# RUN sudo chmod +x src/anygrasp_sdk/license_registration/license_checker
# RUN cd src/anygrasp_sdk/license_registration
# RUN sudo src/anygrasp_sdk/license_registration/license_checker -f
# # RUN cd src/anygrasp_sdk/license_registration && sudo ./license_checker -f

RUN rm -rf /home/$USERNAME/.ros \
    && rm -rf src \
    && mkdir -p src

COPY franka_entrypoint.sh /franka_entrypoint.sh
RUN sudo chmod +x /franka_entrypoint.sh

# Set the default shell to bash and the workdir to the source directory
SHELL [ "/bin/bash", "-c" ]
ENTRYPOINT [ "/franka_entrypoint.sh" ]
CMD [ "/bin/bash" ]
WORKDIR /ros2_ws