#!/bin/bash

# Clone Franka dependencies into the workspace
vcs import /ros2_ws/src/franka_ros2 < /ros2_ws/src/dependencies.repos --recursive --skip-existing
echo "AnyGrasp License: $(/ros2_ws/src/anygrasp_sdk/license_registration/license_checker -f)"

exec "$@"