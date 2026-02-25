# Using Docker Compose

1. `sudo docker-compose build`
2. `sudo docker-compose up -d`
3. `sudo docker-compose exec franka_ros2 bash`

# Running the program

Outside the docker container,

1. `xhost +local:root`
2. copy the `descriptions/` xacro files to `franka_description/`

Inside the docker container,

2. `colcon build --symlink-install`
3. `source install/setup.bash`
4. `ros2 launch hts_robotics hts_bringup.launch.py`

# Finding the Machine ID

1. clone the anygrasp_sdk
2. enter the required mac address in `docker-compose.yml`
3. inside the docker container:

`wget http://archive.ubuntu.com/ubuntu/pool/main/o/openssl/libssl1.1_1.1.1f-1ubuntu2_amd64.deb`
`sudo dpkg -i libssl1.1_1.1.1f-1ubuntu2_amd64.deb`

4. inside `anygrasp_sdk/license_registration`, run `./license_checker -f` to extract the machine ID

# Recording video:

`ffmpeg -f x11grab -video_size <3840x2160> -framerate 30 -i <:0.0+2880,0> -pix_fmt yuv420p <video.mp4>`

