ros2 action send_goal compute_grasp_validity hts_msgs/action/ComputeGraspValidity "{grasp_pose: {position: {x: $1, y: $2, z: $3}, orientation: {x: $4, y: $5, z: $6, w: $7}}, goal_x: $8, goal_y: $9, goal_z: $10, target_id: $11}" --feedback

geometry_msgs/Pose grasp_pose
float32 goal_x
float32 goal_y
float32 goal_z
int64 target_id