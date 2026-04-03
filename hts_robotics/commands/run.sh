ros2 launch hts_robotics hts_bringup.launch.py | grep -vi -E \
"\
\[rcl\]\
|\[rcl_action\]\
|\[rclcpp_action\]\
|\[hts_node.moveit.kinematics.kdl_kinematics_plugin\]\
|\[moveit.moveit.kinematics.kdl_kinematics_plugin\]\
|\[moveit.moveit.core.planning_scene\]: Adding planning scene diff\
|\[moveit.moveit.ros.planning_scene_monitor\]: scene update\
|\[moveit.moveit.ros.planning_scene_monitor\]: robot state update\
|\[hts_node.moveit.ros.planning_scene_monitor\]: robot state update\
|Received status for unknown goal.\
"