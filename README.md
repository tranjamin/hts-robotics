<h1 style="font-size: 3em;">HTS Robotics</h1>

> **Note:** hts-robotics is not officially supported on Windows.

#### Table of Contents
- [About](#about)
- [Setup](#setup)
  - [Local Machine Installation](#local-machine-installation)
  - [Docker Container Installation](#docker-container-installation)
- [Test the Setup](#test-the-setup)
- [Troubleshooting](#troubleshooting)
  - [libfranka: UDP receive: Timeout error](#libfranka-udp-receive-timeout-error)
- [Contributing](#contributing)
- [License](#license)
- [Contact](#contact)

# About

The **hts-robotics** repository develops a system for the Franka Research 3 robot targeted towards enabling high throughput synthesis for chemical laboratories.

# Setup

This repository is designed to be run within a Docker container.

1. **Initialise any submodules**

```bash
git clone git@github.com:tranjamin/hts-robotics.git
cd hts-robotics
git submodule init && git submodule update --remote

# copy anygrasp .so files (change depending on python version)
cp anygrasp_sdk/grasp_detection/gsnet_versions/gsnet.cpython-310-x86_64-linux-gnu.so hts_anygrasp/hts_anygrasp/gsnet.so
cp anygrasp_sdk/license_registration/lib_cxx_versions/lib_cxx.cpython-310-x86_64-linux-gnu.so hts_anygrasp/hts_anygrasp/lib_cxx.so

# --- copy model weights to hts_anygrasp/hts_anygrasp/log
# --- copy license to hts_anygrasp/hts_anygrasp/license

cd anygrasp_sdk && touch COLCON_IGNORE
cd dependencies && git clone https://github.com/graspnet/graspnetAPI.git

git clone https://github.com/realsenseai/librealsense.git
git clone https://github.com/realsenseai/realsense-ros.git
cd realsense-ros/realsense2_description && touch COLCON_IGNORE
```

This repository contains a `.repos` file that helps you clone the required dependencies for Franka ROS 2.

2. **Prepare to use a docker container**

```bash
xhost +local:root
echo -e "USER_UID=$(id -u $USER)\nUSER_GID=$(id -g $USER)" > .env
```

2. **Install CUDA Container Toolkit**

```bash
sudo apt-get update && sudo apt-get install -y --no-install-recommends \
    ca-certificates \
    curl \
    gnupg2
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
    && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt-get update
export NVIDIA_CONTAINER_TOOLKIT_VERSION=1.18.2-1
sudo apt-get install -y \
    nvidia-container-toolkit=${NVIDIA_CONTAINER_TOOLKIT_VERSION} \
    nvidia-container-toolkit-base=${NVIDIA_CONTAINER_TOOLKIT_VERSION} \
    libnvidia-container-tools=${NVIDIA_CONTAINER_TOOLKIT_VERSION} \
    libnvidia-container1=${NVIDIA_CONTAINER_TOOLKIT_VERSION}
```

3. **Build the container**

```bash
sudo docker-compose build
```

4. **Run the container**

```bash
sudo docker-compose up -d
```

5. **Open the shell inside the container**

```bash
sudo docker-compose exec hts_robotics bash
```

6. **Clone any dependencies**

```bash
vcs import src < src/franka_ros2/franka.repos --recursive --skip-existing
```

7. **Build the workspace**

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

8. **Launch the program**

```bash
ros2 launch hts_robotics hts_bringup.launch.py
```

**To record any video:**

```bash
ffmpeg -f x11grab -video_size <3840x2160> -framerate 30 -i <:0.0+2880,0> -pix_fmt yuv420p <video.mp4>
```

# Set up a Conda Environment for Anygrasp

1. Install Miniconda:

```bash
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
bash Miniconda3-latest-Linux-x86_64.sh
rm Miniconda3-latest-Linux-x86_64.sh
```

2. Install GCC-11:

```bash
sudo apt install gcc-11 g++-11
export CC=gcc-11 && CXX=g++-11 # either set this in the ~/.bashrc file or manually export it every time
```

3. Install NVIDIA Drivers:

```bash
# first, clean the workspace from any CUDA artifacts
sudo apt install nvidia-driver-590 # you can browse other driver options with ubuntu-drivers devices
sudo reboot
nvidia-smi # verify that the drivers are installed
```

4. Install NVIDIA Toolkit (11.8):

For instructions on how to install other toolkit versions, see https://developer.nvidia.com/cuda-toolkit-archive

```bash
# install prerequisites
vim /etc/apt/sources.list.d/ubuntu.sources
```
Append to the bottom:
```
Types: deb
URIs: http://old-releases.ubuntu.com/ubuntu/
Suites: lunar
Components: universe
Signed-By: /usr/share/keyrings/ubuntu-archive-keyring.gpg
```
Then,

```bash
# install toolkit
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-ubuntu2204.pin
sudo mv cuda-ubuntu2204.pin /etc/apt/preferences.d/cuda-repository-pin-600
wget https://developer.download.nvidia.com/compute/cuda/11.8.0/local_installers/cuda-repo-ubuntu2204-11-8-local_11.8.0-520.61.05-1_amd64.deb
sudo dpkg -i cuda-repo-ubuntu2204-11-8-local_11.8.0-520.61.05-1_amd64.deb
sudo cp /var/cuda-repo-ubuntu2204-11-8-local/cuda-*-keyring.gpg /usr/share/keyrings/
sudo apt update
sudo apt install cuda-toolkit-11-8 # DO NOT `sudo apt install cuda` if you do not wish to change your drivers
```

```bash
vim ~/.bashrc
```
Append to the bottom:
```bash
export CUDA_HOME=/usr/local/cuda-11.8
export PATH=/usr/local/cuda-11.8/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-11.8/lib64:$LD_LIBRARY_PATH
```

Then,
```bash
source ~/.bashrc # to reload
nvcc --version # verify toolkit install
```

2. Create Conda Environment

Using the `environment.yaml` file:

```bash
conda env create -n anygrasp -f anygrasp_env.yaml
conda activate anygrasp
```

Without the  `environment.yaml` file:
```bash
conda create -n py3.8 python=3.8
conda activate py3.8
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu118 # replace cu118 with whichever version of CUDA you have, look at the pytorch website for more details
conda install openblas-devel -c anaconda
```

3. Register the license

```bash
wget http://archive.ubuntu.com/ubuntu/pool/main/o/openssl/libssl1.1_1.1.1f-1ubuntu2_amd64.deb
sudo dpkg -i libssl1.1_1.1.1f-1ubuntu2_amd64.deb
rm libssl1.1_1.1.1f-1ubuntu2_amd64.deb
./license_checker -f ## use this ID to request a license from AnyGrasp
```

4. Download AnyGrasp Dependencies

```bash
cd anygrasp_sdk
mkdir dependencies && cd dependencies
git clone git@github.com:chenxi-wang/MinkowskiEngine.git
conda install openblas-devel -c anaconda
export CUDA_HOME=/usr/bin
python setup.py install --blas_include_dirs=${CONDA_PREFIX}/include --blas_library_dirs=${CONDA_PREFIX}/lib --blas=openblas
pip install graspnetapi==1.2.11
pip install numpy==1.23.5
cd ../../pointnet2
python setup.py install
```

New rules for downloading AnyGrasp:
```bash
# add local to PYTHONPATH
pip install -U git+https://github.com/NVIDIA/MinkowskiEngine --no-deps

cd graspnetAPI/
pip install numpy==1.23.4 opencv-python scikit-image scipy open3d tqdm Pillow
python3 setup.py install --user

cd pointnet2/
sudo python3 -m pip install torch torchvision --indux-url https://download.pytorch.org/whl/cu118
sudo pip install .
```

Additional Installs:
```bash
sudo apt install libusb-1.0-0-dev libudev-dev pkg-config libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev cmake
sudo apt install freeglut3-dev libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev libgl1-mesa-dev mesa-common-dev
sudo apt install libgl1-mesa-dev mesa-common-dev

cd src/librealsense
rm -rf build && mkdir build && cd build
cmake ..
cmake --build .

sudo apt install ros-humble-octomap-server
```

Instructions to install stomp:
```bash
sudo apt remove ros-humble-moveit*
git clone https://github.com/moveit/moveit2.git -b main
for repo in moveit2/moveit2.repos $(f="moveit2/moveit2_$ROS_DISTRO.repos"; test -r $f && echo $f); do vcs import < "$repo"; done
rosdep install -r --from-paths moveit_msgs moveit_resources moveit2 --ignore-src --rosdistro $ROS_DISTRO -y
colcon build --symlink-install moveit_msgs
colcon build --symlink-install moveit_**
colcon build --symlink-install moveit-common moveit-core
source install/setup.bash
colcon build --symlink-install
```