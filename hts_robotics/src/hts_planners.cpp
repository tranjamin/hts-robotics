#include "hts_node.hpp"

#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>

#include <moveit/planning_interface/planning_interface.hpp>
#include <moveit/planning_pipeline/planning_pipeline.hpp>

#include <moveit/robot_state/robot_state.hpp>
#include <moveit/robot_model/joint_model_group.hpp>

#include "geometry_msgs/msg/pose.hpp"
// #include "trajectory_msgs/msg/joint_trajectory.hpp"

#include <tf2_eigen/tf2_eigen.hpp>

moveit::core::MoveItErrorCode hts_node::plan_pickup(const moveit::core::RobotState& start_state, const geometry_msgs::msg::Pose& target_pose, moveit::planning_interface::MoveGroupInterface::Plan &plan) {
    move_group_interface_->getCurrentState(10.0);
    move_group_interface_->setStartState(start_state);

    auto current_pose = move_group_interface_->getCurrentPose();
    move_group_interface_->clearPathConstraints();
    move_group_interface_->setPoseTarget(target_pose);

    RCLCPP_INFO(this->get_logger(), "Calculating Axis Alignment...");
    Eigen::Quaterniond q(current_pose.pose.orientation.w, current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z);
    Eigen::Vector3d reference_axis(0, 0, 1);
    Eigen::Vector3d translated_axis = q * Eigen::Vector3d(1, 0, 0);
    double alignment = translated_axis.dot(reference_axis);
    RCLCPP_INFO(this->get_logger(), "Axis Alignment is %f", alignment);


    RCLCPP_INFO(this->get_logger(), "Computing Path using OMPL...");
    moveit::core::MoveItErrorCode ompl_status = move_group_interface_->plan(plan);
    RCLCPP_INFO(this->get_logger(), "OMPL finished with error code %d", ompl_status.val);

    if (ompl_status == moveit::core::MoveItErrorCode::SUCCESS) {
      float trajectory_length_pickup = (float) compute_trajectory_length_(plan.trajectory.joint_trajectory);
      RCLCPP_INFO(this->get_logger(), "Unrefined length is %.5f", trajectory_length_pickup);
    } else {
      return ompl_status;
    }

    this->print_robot_state(plan.start_state);
    // const auto& pt = plan.trajectory.joint_trajectory.points.front();
    // for (size_t i = 0; i < pt.positions.size(); ++i) {
    //   RCLCPP_INFO(this->get_logger(), "traj start %s: %f",
    //     plan.trajectory.joint_trajectory.joint_names[i].c_str(),
    //     pt.positions[i]);
    // }

    RCLCPP_INFO(this->get_logger(), "Planned from OMPL. Now refining with STOMP");

    moveit::core::MoveItErrorCode stomp_status = this->refine_path_with_stomp(plan);

    if (stomp_status == moveit::core::MoveItErrorCode::SUCCESS) {
      float trajectory_length_pickup = (float) this->compute_trajectory_length_(plan.trajectory.joint_trajectory);
      RCLCPP_INFO(this->get_logger(), "Refined length is %.5f", trajectory_length_pickup);
    } else {
      return ompl_status;
    }

    // const auto& pt2 = plan.trajectory.joint_trajectory.points.front();
    //   for (size_t i = 0; i < pt2.positions.size(); ++i) {
    //     RCLCPP_INFO(this->get_logger(), "traj start %s: %f",
    //       plan.trajectory.joint_trajectory.joint_names[i].c_str(),
    //       pt2.positions[i]);
    //   }


    return ompl_status;
}

moveit::core::MoveItErrorCode hts_node::plan_move(const moveit::core::RobotState& start_state, const geometry_msgs::msg::Pose& start_pose, const geometry_msgs::msg::Pose& target_pose, moveit::planning_interface::MoveGroupInterface::Plan &plan) {
    move_group_interface_->setStartState(start_state);
      
    // Apply orientation constraints
    moveit_msgs::msg::OrientationConstraint orientation_constraint;
    orientation_constraint.header.frame_id = move_group_interface_->getPoseReferenceFrame();
    orientation_constraint.link_name = move_group_interface_->getEndEffectorLink();
    orientation_constraint.orientation = start_pose.orientation;
    orientation_constraint.absolute_x_axis_tolerance = 3.142;
    orientation_constraint.absolute_y_axis_tolerance = 0.6;
    orientation_constraint.absolute_z_axis_tolerance = 0.6;
    orientation_constraint.weight = 1.0;
    // orientation_constraint.parameterization = orientation_constraint.ROTATION_VECTOR;
      
    moveit_msgs::msg::Constraints all_constraints;
    all_constraints.name = "Move Constraint";
    all_constraints.orientation_constraints.emplace_back(orientation_constraint);

    this->log_pose(start_pose, "Start Pose");
    this->log_pose(target_pose, "Target Pose");

    // move_group_interface_->setPositionTarget(target_pose.position.x, target_pose.position.y, target_pose.position.z);
    move_group_interface_->setPoseTarget(target_pose);
    double orig_goal_tolerance = move_group_interface_->getGoalOrientationTolerance();
    move_group_interface_->setGoalOrientationTolerance(3.142);

    move_group_interface_->clearPathConstraints();
    move_group_interface_->setPathConstraints(all_constraints);
    RCLCPP_INFO(this->get_logger(), "Applied orientation constraints to planning scene.");

    RCLCPP_INFO(this->get_logger(), "Displaying some info about the move...");
    planning_interface::MotionPlanResponse motion_plan_response;
    planning_interface::MotionPlanRequest motion_plan_request;
    bool ompl_status;
    moveit::core::MoveItErrorCode err_code;

    #define USE_SELF_PIPELINE false

    if (!USE_SELF_PIPELINE) {
        RCLCPP_INFO(this->get_logger(), "Computing Path using OMPL built in pipeline...");
        ompl_status = (err_code = move_group_interface_->plan(plan)) == moveit::core::MoveItErrorCode::SUCCESS;
    } else {
        move_group_interface_->constructMotionPlanRequest(motion_plan_request);
        motion_plan_request.goal_constraints[0].name = "Move Goal Constraint";
        motion_plan_request.goal_constraints[0].orientation_constraints[0].absolute_x_axis_tolerance = 3.142;
        motion_plan_request.goal_constraints[0].orientation_constraints[0].absolute_y_axis_tolerance = 0.1;
        motion_plan_request.goal_constraints[0].orientation_constraints[0].absolute_z_axis_tolerance = 0.1;
        
        motion_plan_request.path_constraints.name = "Move Path Constraint";
        motion_plan_request.path_constraints.orientation_constraints[0].absolute_x_axis_tolerance = 3.142;
        motion_plan_request.path_constraints.orientation_constraints[0].absolute_y_axis_tolerance = 0.1;      
        motion_plan_request.path_constraints.orientation_constraints[0].absolute_z_axis_tolerance = 0.1;     

        this->printMotionPlanRequestFull(motion_plan_request);

        RCLCPP_INFO(this->get_logger(), "======================= Testing Selecting HTS IK Constraint Sampler =======================");
        auto scene = planning_scene_monitor_->getPlanningScene();
        moveit_msgs::msg::Constraints constr = motion_plan_request.goal_constraints[0];
        constraint_samplers::ConstraintSamplerPtr chosen_sampler = sampler_manager->selectSampler(scene, "fr3_arm", constr);
        RCLCPP_INFO(this->get_logger(), "Selected the sampler %s.", chosen_sampler->getName().c_str());
        RCLCPP_INFO(this->get_logger(), "============================================================================================");

        planning_scene_monitor::LockedPlanningSceneRO locked_scene(planning_scene_monitor_);

        RCLCPP_INFO(this->get_logger(), "======================= Computing Path using OMPL self pipeline =======================");
        ompl_status = planning_pipeline_ompl->generatePlan(locked_scene, motion_plan_request, motion_plan_response);
        RCLCPP_INFO(this->get_logger(), "============================================================================================");

        if (ompl_status) {
          	moveit_msgs::msg::RobotTrajectory traj;
          	motion_plan_response.trajectory->getRobotTrajectoryMsg(traj);
          	plan.trajectory.joint_trajectory = traj.joint_trajectory;
          	plan.trajectory.multi_dof_joint_trajectory = traj.multi_dof_joint_trajectory;
        }
        err_code = motion_plan_response.error_code;
        plan.planning_time = motion_plan_response.planning_time;
        plan.start_state = motion_plan_response.start_state;
    }

    move_group_interface_->clearPathConstraints();
    move_group_interface_->setGoalOrientationTolerance(orig_goal_tolerance);

    if (ompl_status == moveit::core::MoveItErrorCode::SUCCESS) {
        float trajectory_length_pickup = (float) this->compute_trajectory_length_(plan.trajectory.joint_trajectory);
        RCLCPP_INFO(this->get_logger(), "Unrefined length is %.5f", trajectory_length_pickup);
    } else {
        return ompl_status;
    }

    const auto& pt = plan.trajectory.joint_trajectory.points.front();
    for (size_t i = 0; i < pt.positions.size(); ++i) {
        RCLCPP_INFO(this->get_logger(), "traj start %s: %f",
    	    plan.trajectory.joint_trajectory.joint_names[i].c_str(),
        	pt.positions[i]);
    }


	// RCLCPP_INFO(this->get_logger(), "Planned from OMPL. Now refining with STOMP");
	// moveit::core::MoveItErrorCode stomp_status = refine_path_with_stomp(plan);


	// if (stomp_status == moveit::core::MoveItErrorCode::SUCCESS) {
	//   float trajectory_length_pickup = (float) compute_trajectory_length_(plan.trajectory.joint_trajectory);
	//   RCLCPP_INFO(this->get_logger(), "Refined length is %.5f", trajectory_length_pickup);
	// } else {
	//   return ompl_status;
	// }

	// const auto& pt2 = plan.trajectory.joint_trajectory.points.front();
	//   for (size_t i = 0; i < pt2.positions.size(); ++i) {
	//     RCLCPP_INFO(this->get_logger(), "traj start %s: %f",
	//       plan.trajectory.joint_trajectory.joint_names[i].c_str(),
	//       pt2.positions[i]);
	//   }

	return ompl_status;
}

bool hts_node::compute_IK_manually(const geometry_msgs::msg::Pose& target_pose, moveit::core::RobotState& ik_state) {
    std::string group_name = move_group_interface_->getName();
    const moveit::core::JointModelGroup* joint_model_group = planning_scene_monitor_->getRobotModel()->getJointModelGroup(group_name);

    return ik_state.setFromIK(joint_model_group, target_pose, move_group_interface_->getEndEffectorLink(), 1.0);
}

moveit::core::MoveItErrorCode hts_node::refine_path_with_stomp(moveit::planning_interface::MoveGroupInterface::Plan &plan) {
    return moveit::core::MoveItErrorCode::SUCCESS;

    planning_interface::MotionPlanResponse motion_plan_response;
    planning_interface::MotionPlanRequest motion_plan_request;
    move_group_interface_->constructMotionPlanRequest(motion_plan_request);

    motion_plan_request.start_state = plan.start_state;
    motion_plan_request.start_state.is_diff = false;

    moveit_msgs::msg::GenericTrajectory generic_trajectory;
    generic_trajectory.joint_trajectory.push_back(plan.trajectory.joint_trajectory);
    motion_plan_request.reference_trajectories.clear();
    motion_plan_request.reference_trajectories.push_back(generic_trajectory);
    motion_plan_request.pipeline_id = "stomp";

    planning_scene_monitor::LockedPlanningSceneRO locked_scene(planning_scene_monitor_);

    bool stomp_status = planning_pipeline->generatePlan(locked_scene, motion_plan_request, motion_plan_response);

    if (stomp_status) {
    moveit_msgs::msg::RobotTrajectory stomp_traj;
    motion_plan_response.trajectory->getRobotTrajectoryMsg(stomp_traj);

    // plan.trajectory.joint_trajectory = stomp_traj.joint_trajectory;
    // plan.trajectory.multi_dof_joint_trajectory = stomp_traj.multi_dof_joint_trajectory;
    }

    RCLCPP_INFO(this->get_logger(), "Finished Planning STOMP. Result success is %d with error code %d", stomp_status, motion_plan_response.error_code.val);
    return motion_plan_response.error_code;
}
