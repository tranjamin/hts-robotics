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