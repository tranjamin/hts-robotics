#include "hts_node.hpp"

#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>

#include <moveit/planning_interface/planning_interface.hpp>
#include <moveit/planning_pipeline/planning_pipeline.hpp>

#include <moveit/robot_state/robot_state.hpp>
#include <moveit/robot_model/joint_model_group.hpp>

void hts_node::print_robot_state(moveit_msgs::msg::RobotState msg) {
    // --- Joint states ---
    const auto& names = msg.joint_state.name;
    const auto& positions = msg.joint_state.position;

    RCLCPP_INFO(this->get_logger(), "---- RobotState ----");

    for (size_t i = 0; i < names.size(); ++i) {
    double val = (i < positions.size()) ? positions[i] : 0.0;
    RCLCPP_INFO(this->get_logger(), "  %s: %f", names[i].c_str(), val);
    }

    RCLCPP_INFO(this->get_logger(), "--------------------");
}

void hts_node::print_robot_state(moveit::core::RobotState state) {
    // --- Joint positions ---
    std::vector<std::string> joint_names;
    std::vector<double> joint_values;

    joint_names = state.getVariableNames();
    joint_values.resize(joint_names.size());
    state.copyJointGroupPositions(state.getRobotModel()->getJointModelGroupNames()[0], joint_values);

    RCLCPP_INFO(this->get_logger(), "---- RobotState ----");

    for (size_t i = 0; i < joint_names.size(); ++i) {
    RCLCPP_INFO(this->get_logger(), "  %s: %f", joint_names[i].c_str(), joint_values[i]);
    }

    // --- Attached objects ---
    // const auto& attached_bodies = state.getAttachedBodies();
    // RCLCPP_INFO(this->get_logger(), "Attached bodies: %zu", attached_bodies.size());

    // for (const auto* body : attached_bodies)
    // {
    //   RCLCPP_INFO(this->get_logger(), "  - %s (link: %s)",
    //       body->getName().c_str(),
    //       body->getAttachedLinkName().c_str());
    // }

    RCLCPP_INFO(this->get_logger(), "--------------------");
}

double hts_node::compute_trajectory_length_(trajectory_msgs::msg::JointTrajectory trajectory) {
    double total_length = 0;
    for (size_t i=1; i < trajectory.points.size(); i++) {
        const std::vector<double> &prev_joints = trajectory.points[i - 1].positions;
        const std::vector<double> &curr_joints = trajectory.points[i].positions;
        for (size_t j = 0; j < prev_joints.size(); ++j) {
            total_length += std::sqrt(pow(curr_joints[j] - prev_joints[j], 2));
        }
    }

    return total_length;
}

void hts_node::printCollisionContacts(const collision_detection::CollisionResult& res, const rclcpp::Logger& logger) {
    if (!res.collision) {
        RCLCPP_INFO(logger, "No collision detected.");
        return;
    }

    RCLCPP_WARN(logger, "Collision detected! Number of contact pairs: %zu", res.contacts.size());

    for (const auto& contact_pair : res.contacts) {
        const std::string& body_1 = contact_pair.first.first;
        const std::string& body_2 = contact_pair.first.second;

        const std::vector<collision_detection::Contact>& contacts = contact_pair.second;

        RCLCPP_WARN(logger, "Collision between: [%s] and [%s] (%zu contact points)",
                    body_1.c_str(), body_2.c_str(), contacts.size());

        for (size_t i = 0; i < contacts.size(); ++i) {
            const auto& c = contacts[i];

            RCLCPP_INFO(logger, "  Contact %zu:", i);
            RCLCPP_INFO(logger, "    Position: [%.4f, %.4f, %.4f]",
                        c.pos.x(), c.pos.y(), c.pos.z());

            RCLCPP_INFO(logger, "    Normal:   [%.4f, %.4f, %.4f]",
                        c.normal.x(), c.normal.y(), c.normal.z());

            RCLCPP_INFO(logger, "    Depth:    %.6f", c.depth);
        }
    }
}

void hts_node::printConstraints(const moveit_msgs::msg::Constraints& c) {
    // --- Position Constraints ---
    for (size_t i = 0; i < c.position_constraints.size(); ++i) {
        const auto& pc = c.position_constraints[i];
        RCLCPP_INFO(this->get_logger(), "    [PositionConstraint %zu] link: %s", i, pc.link_name.c_str());
        RCLCPP_INFO(this->get_logger(), "      frame: %s", pc.header.frame_id.c_str());
        RCLCPP_INFO(this->get_logger(), "      target point offset: (%f, %f, %f)",
                    pc.target_point_offset.x,
                    pc.target_point_offset.y,
                    pc.target_point_offset.z);
        RCLCPP_INFO(this->get_logger(), "      tolerance: x=%f y=%f z=%f",
                    pc.constraint_region.primitives[0].dimensions[0],
                    pc.constraint_region.primitives[0].dimensions[0],
                    pc.constraint_region.primitives[0].dimensions[0]);
    }

    // --- Orientation Constraints ---
    for (size_t i = 0; i < c.orientation_constraints.size(); ++i) {
        const auto& oc = c.orientation_constraints[i];
        RCLCPP_INFO(this->get_logger(), "    [OrientationConstraint %zu] link: %s", i, oc.link_name.c_str());
        RCLCPP_INFO(this->get_logger(), "      frame: %s", oc.header.frame_id.c_str());
        RCLCPP_INFO(this->get_logger(), "      orientation: (%f, %f, %f, %f)",
                    oc.orientation.x, oc.orientation.y, oc.orientation.z, oc.orientation.w);
        RCLCPP_INFO(this->get_logger(), "      tolerances: x=%f y=%f z=%f, weight=%f",
                    oc.absolute_x_axis_tolerance, oc.absolute_y_axis_tolerance,
                    oc.absolute_z_axis_tolerance, oc.weight);
    }

    // --- Joint Constraints ---
    for (size_t i = 0; i < c.joint_constraints.size(); ++i) {
        const auto& jc = c.joint_constraints[i];
        RCLCPP_INFO(this->get_logger(), "    [JointConstraint %zu] joint: %s", i, jc.joint_name.c_str());
        RCLCPP_INFO(this->get_logger(), "      position: %f, tolerance: %f, weight: %f",
                    jc.position, jc.tolerance_above, jc.weight);
    }

    // --- Visibility Constraints ---
    // for (size_t i = 0; i < c.visibility_constraints.size(); ++i)
    // {
    //     const auto& vc = c.visibility_constraints[i];
    //     RCLCPP_INFO(node->get_logger(), "    [VisibilityConstraint %zu] sensor: %s, target: %s", i,
    //                 vc.sensor_frame.c_str(), vc.target_frame.c_str());
    //     RCLCPP_INFO(node->get_logger(), "      cone_angle: %f, max_range: %f, weight: %f",
    //                 vc.cone_angle, vc.max_range, vc.weight);
    // }
}

void hts_node::printMotionPlanRequestFull(const moveit_msgs::msg::MotionPlanRequest& request) {
    RCLCPP_INFO(this->get_logger(), "======================= MotionPlanRequest =======================");

    // --- Goal Constraints ---
    RCLCPP_INFO(this->get_logger(), "Goal Constraints (%zu):", request.goal_constraints.size());
    for (size_t i = 0; i < request.goal_constraints.size(); ++i)
    {
        RCLCPP_INFO(this->get_logger(), "  Goal %zu:", i);
        printConstraints(request.goal_constraints[i]);
    }

    // --- Path Constraints ---
    RCLCPP_INFO(this->get_logger(), "Path Constraints:");
    printConstraints(request.path_constraints);

    // --- Trajectory Constraints ---
    RCLCPP_INFO(this->get_logger(), "Trajectory Constraints (%zu):", request.trajectory_constraints.constraints.size());
    for (size_t i = 0; i < request.trajectory_constraints.constraints.size(); ++i)
    {
        RCLCPP_INFO(this->get_logger(), "  Constraint %zu:", i);
        printConstraints(request.trajectory_constraints.constraints[i]);
    }

    // // --- Reference Trajectories ---
    // RCLCPP_INFO(this->get_logger(), "Reference Trajectories (%zu):", request.reference_trajectories.size());
    // for (size_t i = 0; i < request.reference_trajectories.size(); ++i)
    // {
    //     const auto& ref = request.reference_trajectories[i];
    //     RCLCPP_INFO(this->get_logger(), "  Reference trajectory %zu:", i);
    //     RCLCPP_INFO(this->get_logger(), "    Trajectory points: %zu", ref.trajectory.joint_trajectory.points.size());
    // }

    RCLCPP_INFO(this->get_logger(), "============================================================================================");
}

void hts_node::log_planning_details() {
    std::vector<moveit_msgs::msg::PlannerInterfaceDescription> desc;
    move_group_interface_->getInterfaceDescriptions(desc);

    RCLCPP_INFO(this->get_logger(), "Loaded planning pipelines and planners:");
    for (const auto &pipeline : desc) {
        RCLCPP_INFO(this->get_logger(), "Pipeline name: %s", pipeline.name.c_str());
        for (const auto &planner_id : pipeline.planner_ids) {
            RCLCPP_INFO(this->get_logger(), "  Planner ID: %s", planner_id.c_str());

            std::map<std::string, std::string> params = move_group_interface_->getPlannerParams(planner_id.c_str(), "move_group");
            for (const auto& [key, value] : params) {
                RCLCPP_INFO(this->get_logger(), "    Move Group Param: %s = %s", key.c_str(), value.c_str());
            }

            std::map<std::string, std::string> params2 = move_group_interface_->getPlannerParams(planner_id.c_str(), "fr3_arm");
            for (const auto& [key, value] : params2) {
                RCLCPP_INFO(this->get_logger(), "    Fr3 Arm Param: %s = %s", key.c_str(), value.c_str());
            }
        }
    }

    moveit_msgs::msg::PlannerInterfaceDescription default_desc;
    move_group_interface_->getInterfaceDescription(default_desc);
    RCLCPP_INFO(this->get_logger(), "Pipeline name: %s", default_desc.name.c_str());
    for (const auto &planner_id : default_desc.planner_ids) {
        RCLCPP_INFO(this->get_logger(), "  Planner ID: %s", planner_id.c_str());
    }

    RCLCPP_INFO(this->get_logger(), "default planning pipeline id: %s", move_group_interface_->getDefaultPlanningPipelineId().c_str());
    RCLCPP_INFO(this->get_logger(), "default planner id: %s", move_group_interface_->getDefaultPlannerId().c_str());
    RCLCPP_INFO(this->get_logger(), "current planner id: %s", move_group_interface_->getPlannerId().c_str());
}

void hts_node::log_pose(const geometry_msgs::msg::Pose &pose, const char* descriptor="") {
    RCLCPP_INFO(this->get_logger(), "%s Position is (%.2f, %.2f, %.2f)", descriptor, pose.position.x, pose.position.y, pose.position.z);
    RCLCPP_INFO(this->get_logger(), "%s Quaternion is (%.2f, %.2f, %.2f, %.2f)", descriptor, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
}