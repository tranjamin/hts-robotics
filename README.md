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
