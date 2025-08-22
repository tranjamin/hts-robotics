# Robots for High-Throughput Synthesis

## Requirements
- Docker
- Linux, or windows with a compatible X-server (for GUIs)

## Usage
- Clone the repositories and initialise the submodules
- To generate an image, run `docker build -f [ros1_dockerfile | ros2_dockerfile] -t <tag-name> .`
- To run an image, run `docker run -it <tag-name>`
- Execute ROS commands in the shell