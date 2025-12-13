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
```

This repository contains a `.repos` file that helps you clone the required dependencies for Franka ROS 2.

2. **Prepare to use a docker container**

```bash
xhost +local:root
echo -e "USER_UID=$(id -u $USER)\nUSER_GID=$(id -g $USER)" > .env
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

**Set up a Conda Environment for Anygrasp**

1. Install Miniconda:

```bash
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
bash Miniconda3-latest-Linux-x86_64.sh
rm Miniconda3-latest-Linux-x86_64.sh
```

2. Create Conda Environment

```bash
conda env create -n anygrasp -f anygrasp_env.yaml
conda activate anygrasp

```

3. Register the license

```bash
wget http://archive.ubuntu.com/ubuntu/pool/main/o/openssl/libssl1.1_1.1.1f-1ubuntu2_amd64.deb
sudo dpkg -i libssl1.1_1.1.1f-1ubuntu2_amd64.deb
rm libssl1.1_1.1.1f-1ubuntu2_amd64.deb
./license_checker -f ## use this ID to request a license from AnyGrasp
```

4. Find the CUDA Version

```bash
sudo apt install nvidia-utils-390
nvidia-smi ## see what CUDA version you are running
```

5. Install CUDA Dependencies
```bash
sudo apt install nvidia-cuda-toolkit
```

6. Install PyTorch

```bash
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu115 # replace cu115 with which ever version of CUDA you have, or with cpu for no cpu
```

7. Download AnyGrasp Dependencies

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



