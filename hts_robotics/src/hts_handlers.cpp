#include "hts_node.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>

#include <moveit/robot_state/robot_state.hpp>
#include <moveit/robot_model/joint_model_group.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <chrono>

// flags to enable or disable steps
#define RUN_IK_PICKUP true
#define RUN_IK_MOVE false
#define RUN_COLLISIONS_PICKUP false
#define RUN_COLLISIONS_MOVE false

#define DEFAULT_PICKUP_TIME 10.0
#define DEFAULT_MOVE_TIME 10.0

void hts_node::get_object_position(const std::shared_ptr<hts_msgs::srv::GetObjectPosition::Request> request, std::shared_ptr<hts_msgs::srv::GetObjectPosition::Response> response) {
    RCLCPP_INFO(this->get_logger(), "Get Object Position Started");
    int object_id = request->object_id;
    auto object_name = "target_" + std::to_string(object_id);

    auto map = planning_scene_interface_->getObjectPoses({object_name});
    if (map.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Could not find object in planning scene");
        response->success = false;
        return;
    }
    geometry_msgs::msg::Pose target_moveit = map.at(object_name);

    response->x = target_moveit.position.x;
    response->y = target_moveit.position.y;
    response->z = target_moveit.position.z;
    response->success = true;
    RCLCPP_INFO(this->get_logger(), "Get Object Position Done");
    return;
}

void hts_node::handle_accepted_compute_grasp_validity_(const std::shared_ptr<rclcpp_action::ServerGoalHandle<CustomActionComputeGraspValidity>> goal_handle) {
    std::thread([this, goal_handle] {            
        RCLCPP_INFO(this->get_logger(), "Planning with planning time %f", goal_handle->get_goal()->planning_time);

        double base_planning_time = goal_handle->get_goal()->planning_time;
        if (base_planning_time == 0.0) base_planning_time = 0.3;

        double pickup_planning_time = base_planning_time;
        // double move_planning_time = base_planning_time * 10 < 1 ? 1 : base_planning_time * 10;
        double move_planning_time = base_planning_time;

        // lock planning scene
        planning_scene_monitor::LockedPlanningSceneRO planning_scene(planning_scene_monitor_);

        // retrieve object id
        auto object_name = "target_" + std::to_string(goal_handle->get_goal()->target_id);

        // result and error code objects
        auto result = std::make_shared<CustomActionComputeGraspValidity::Result>();
        moveit::core::MoveItErrorCode err_code;

        // get the current state within bounds
        std::shared_ptr<moveit::core::RobotState> current_state = move_group_interface_->getCurrentState(10.0);
        current_state->enforceBounds();
        
        // remove all path constraints
        move_group_interface_->clearPathConstraints();
        
        // retrieve the pose of the grasp
        geometry_msgs::msg::Pose grasp_pose = goal_handle->get_goal()->grasp_pose;

        // objects for collision detection
        collision_detection::CollisionRequest collision_request;
        collision_detection::CollisionResult collision_response;
        collision_request.contacts = true;
        collision_request.verbose = true;
        collision_request.distance = true;
        collision_request.detailed_distance = true;

        auto t0 = std::chrono::high_resolution_clock::now();
        auto t1 = std::chrono::high_resolution_clock::now();

        // ---------------- try to do IK for the current grasp pose --------------- //
        if (RUN_IK_PICKUP) {
            RCLCPP_INFO(this->get_logger(), "Computing IK (Pickup)...");
            moveit::core::RobotState computed_ik_pickup(planning_scene->getRobotModel());
            t0 = std::chrono::high_resolution_clock::now();
            if (!this->compute_IK_manually(grasp_pose, computed_ik_pickup)) {
                t1 = std::chrono::high_resolution_clock::now();
                result->pickup_ik_time = ((std::chrono::duration<double, std::milli>) (t1 - t0)).count() / 1000;
                RCLCPP_ERROR(this->get_logger(), "No IK Solution Found (pickup).");
                result->success = true;
                result->is_valid = false;
                result->score = 0.0;
                result->message = "Plan (pickup IK) is not valid";
                result->err_code = moveit::core::MoveItErrorCode::NO_IK_SOLUTION;
                result->err_source = "pickup IK";
                result->err_message = "";
                goal_handle->succeed(result);
                return;
            }
            t1 = std::chrono::high_resolution_clock::now();
            result->pickup_ik_time = ((std::chrono::duration<double, std::milli>) (t1 - t0)).count() / 1000;

            // ---------------- try to do collision detection for the current grasp pose --------------- //
            if (RUN_COLLISIONS_PICKUP) {
                RCLCPP_INFO(this->get_logger(), "Computing Goal Collision (Pickup)...");
                planning_scene->checkCollision(collision_request, collision_response, computed_ik_pickup);

                if (collision_response.collision) {
                    RCLCPP_ERROR(this->get_logger(), "Goal Collision Failed...");
                    printCollisionContacts(collision_response, get_logger());
                    result->success = true;
                    result->is_valid = false;
                    result->score = 0.0;
                    result->message = "Plan (pickup goal collisions) is not valid";
                    result->err_code = moveit::core::MoveItErrorCode::GOAL_IN_COLLISION;
                    result->err_source = "pickup goal collisions";
                    result->err_message = "";
                    goal_handle->succeed(result);
                    return;
                } else {
                    RCLCPP_INFO(this->get_logger(), "Goal Collision Succeeded. Closest distance is %f", collision_response.distance);
                }
            }
        }

        // ---------------- run trajectory generation for the pickup plan --------------- //
        moveit::planning_interface::MoveGroupInterface::Plan pickup_plan;
        t0 = std::chrono::high_resolution_clock::now();
        err_code = this->plan_pickup(*current_state, grasp_pose, pickup_plan, pickup_planning_time);
        t1 = std::chrono::high_resolution_clock::now();
        result->pickup_plan_time = ((std::chrono::duration<double, std::milli>) (t1 - t0)).count() / 1000;

        if (err_code != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Planning (pickup) failed");
            result->success = true;
            result->is_valid = false;
            result->score = 0.0;
            result->message = "Plan (pickup) is not valid";
            result->err_code = err_code.val;
            result->err_source = "pickup";
            result->err_message = err_code.message;
            goal_handle->succeed(result);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Planning (pickup) succeeded");

        // compute trajectory length
        trajectory_msgs::msg::JointTrajectory pickup_joint_trajectory = pickup_plan.trajectory.joint_trajectory;
        float trajectory_length_pickup = (float) compute_trajectory_length_(pickup_joint_trajectory);
        RCLCPP_INFO(this->get_logger(), "Trajectory length (pickup) is %.5f", trajectory_length_pickup);
        
        // set start state for move operation
        moveit::core::RobotState move_start_state(*current_state);
        const moveit::core::JointModelGroup* joint_model_group = move_start_state.getJointModelGroup(move_group_interface_->getName());
        move_start_state.setJointGroupPositions(joint_model_group, pickup_joint_trajectory.points.back().positions);
        move_start_state.update();
        
        // check to make sure that the start state is equal to the grasp pose
        geometry_msgs::msg::Pose actual_grasp_pose = tf2::toMsg(move_start_state.getGlobalLinkTransform(move_group_interface_->getEndEffectorLink()));
        this->log_pose(actual_grasp_pose, "Check: Path is grasping at this pose");
        this->log_pose(grasp_pose, "It is meant to grasp at this pose");

        // create the goal state for the move operation
        geometry_msgs::msg::Pose goal_pose;
        goal_pose.position.x = goal_handle->get_goal()->goal_x;
        goal_pose.position.y = goal_handle->get_goal()->goal_y;
        goal_pose.position.z = goal_handle->get_goal()->goal_z;
        goal_pose.orientation.x = actual_grasp_pose.orientation.x;
        goal_pose.orientation.y = actual_grasp_pose.orientation.y;
        goal_pose.orientation.z = actual_grasp_pose.orientation.z;
        goal_pose.orientation.w = actual_grasp_pose.orientation.w;

        // ---------------- try to do IK for the goal pose --------------- //
        if (RUN_IK_MOVE) {
            RCLCPP_INFO(this->get_logger(), "Computing IK (Move)...");
            moveit::core::RobotState computed_ik_move(planning_scene->getRobotModel());
            t0 = std::chrono::high_resolution_clock::now();
            if (!compute_IK_manually(goal_pose, computed_ik_move)) {
                t1 = std::chrono::high_resolution_clock::now();
                result->move_ik_time = ((std::chrono::duration<double, std::milli>) (t1 - t0)).count() / 1000;
                RCLCPP_ERROR(this->get_logger(), "No IK Solution Found (move).");
                result->success = true;
                result->is_valid = false;
                result->score = 0.0;
                result->message = "Plan (move goal IK) is not valid";
                result->err_code = moveit::core::MoveItErrorCode::GOAL_IN_COLLISION;
                result->err_source = "move goal IK";
                result->err_message = "";
                goal_handle->succeed(result);
                return;
            }
            t1 = std::chrono::high_resolution_clock::now();
            result->move_ik_time = ((std::chrono::duration<double, std::milli>) (t1 - t0)).count() / 1000;

            // ---------------- try to do collision detection for the goal pose --------------- //
            if (RUN_COLLISIONS_MOVE) {
                RCLCPP_INFO(this->get_logger(), "Computing Goal Collision (Pickup)...");
                planning_scene->checkCollision(collision_request, collision_response, computed_ik_move);

                if (collision_response.collision) {
                    RCLCPP_ERROR(this->get_logger(), "Goal Collision Failed...");
                    result->success = true;
                    result->is_valid = false;
                    result->score = 0.0;
                    result->message = "Plan (move collisions) is not valid";
                    result->err_code = moveit::core::MoveItErrorCode::NO_IK_SOLUTION;
                    result->err_source = "move collisions";
                    result->err_message = "";
                    goal_handle->succeed(result);
                    return;
                } else {
                    RCLCPP_INFO(this->get_logger(), "Goal Collision Succeeded. Closest distance is %f", collision_response.distance);
                }
            }
        }
        
        // attach target object to gripper in preparation for move
        if (goal_handle->get_goal()->target_id >= 0) {
            // register_grasped_object(object_name); // we don't need to do this because we don't actually close the gripper
            gripper_interface_->attachObject(object_name);
        }

        // ---------------- run trajectory generation for the move operation --------------- //        
        moveit::planning_interface::MoveGroupInterface::Plan move_plan;
        t0 = std::chrono::high_resolution_clock::now();
        // err_code = this->plan_move_no_refine(move_start_state, grasp_pose, goal_pose, move_plan);
        t1 = std::chrono::high_resolution_clock::now();
        result->move_plan_time = ((std::chrono::duration<double, std::milli>) (t1 - t0)).count() / 1000;

        t0 = std::chrono::high_resolution_clock::now();
        err_code = this->plan_move(move_start_state, grasp_pose, goal_pose, move_plan, move_planning_time);
        t1 = std::chrono::high_resolution_clock::now();
        result->move_refine_time = ((std::chrono::duration<double, std::milli>) (t1 - t0)).count() / 1000 - result->move_refine_time;

        // detach target object
        if (goal_handle->get_goal()->target_id >= 0) {
            gripper_interface_->detachObject(object_name);
            // deregister_grasped_object(object_name); // we don't need to do this because we don't actually close the gripper
        }

        if (err_code != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Planning (move) failed");
            result->success = true;
            result->is_valid = false;
            result->score = 0.0;
            result->message = "Plan (move) is not valid";
            result->err_code = err_code.val;
            result->err_source = "move";
            result->err_message = err_code.message;
            goal_handle->succeed(result);
            return;
        } 

        RCLCPP_INFO(this->get_logger(), "Planning (move) succeeded");

        // compute trajectory length
        trajectory_msgs::msg::JointTrajectory move_joint_trajectory = move_plan.trajectory.joint_trajectory;
        float trajectory_length_move = (float) compute_trajectory_length_(move_joint_trajectory);
        RCLCPP_INFO(this->get_logger(), "Trajectory length (move) is %.5f", trajectory_length_move);

        // if both are valid:
        result->success = true;
        result->is_valid = true;
        result->score = trajectory_length_pickup + trajectory_length_move;
        result->message = "Plan is valid";
        goal_handle->succeed(result);
    }).detach();

}

void hts_node::handle_accepted_close_(const std::shared_ptr<rclcpp_action::ServerGoalHandle<CustomActionClose>> goal_handle) {
    std::thread([this, goal_handle] {
        RCLCPP_INFO(this->get_logger(), "\n\n\n\n--------------- CLOSE CALLBACK ---------------\n\n\n\n");

        std::shared_ptr<moveit::core::RobotState> current_state = gripper_interface_->getCurrentState(10.0);
        current_state->enforceBounds();
        gripper_interface_->setStartState(*current_state);

        gripper_interface_->setNamedTarget("close");
        auto object_name = "target_" + std::to_string(goal_handle->get_goal()->target_id);

        // disable collisions
        planning_scene_monitor::LockedPlanningSceneRW scene(planning_scene_monitor_);
        auto &acm = scene->getAllowedCollisionMatrixNonConst();
        acm.setEntry(object_name, "fr3_hand", true);
        acm.setEntry(object_name, "fr3_leftfinger", true);
        acm.setEntry(object_name, "fr3_rightfinger", true);
        moveit_msgs::msg::PlanningScene ps_msg;
        ps_msg.is_diff = true;
        scene->getPlanningSceneMsg(ps_msg);
        planning_scene_interface_->applyPlanningScene(ps_msg);

        // planning_scene_monitor_->triggerSceneUpdateEvent(
        //   planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE
        // );
        // rclcpp::sleep_for(std::chrono::milliseconds(100));

        // move
        bool success = (gripper_interface_->move() == moveit::core::MoveItErrorCode::SUCCESS);

        // attach object
        if (goal_handle->get_goal()->target_id >= 0) {
            gripper_interface_->attachObject(object_name);
        }

        // log results
        auto result = std::make_shared<CustomActionClose::Result>();
        result->success = success;
        if (success) {
            RCLCPP_INFO(this->get_logger(), "Goal reached successfully");
            result->message = "Goal reached successfully";
            goal_handle->succeed(result);
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal failed");
            result->message = "Goal failed";
            goal_handle->abort(result);
        }

        RCLCPP_INFO(this->get_logger(), "\n\n\n\n--------------- CLOSE CALLBACK END ---------------\n\n\n\n");
    }).detach();
}

void hts_node::handle_accepted_open_(const std::shared_ptr<rclcpp_action::ServerGoalHandle<CustomActionOpen>> goal_handle) {
    std::thread([this, goal_handle] {
        RCLCPP_INFO(this->get_logger(), "\n\n\n\n--------------- OPEN CALLBACK ---------------\n\n\n\n");
        
        auto object_name = "target_" + std::to_string(goal_handle->get_goal()->target_id);

        std::shared_ptr<moveit::core::RobotState> current_state = gripper_interface_->getCurrentState(10.0);
        current_state->enforceBounds();
        gripper_interface_->setStartState(*current_state);

        gripper_interface_->setNamedTarget("open");

        // detach object
        if (goal_handle->get_goal()->target_id >= 0) {
            gripper_interface_->detachObject(object_name);
        }

        // move
        bool success = (gripper_interface_->move() == moveit::core::MoveItErrorCode::SUCCESS);

        // enable collisions
        planning_scene_monitor::LockedPlanningSceneRW scene(planning_scene_monitor_);
        auto &acm = scene->getAllowedCollisionMatrixNonConst();
        acm.setEntry(object_name, "fr3_hand", false);
        acm.setEntry(object_name, "fr3_leftfinger", false);
        acm.setEntry(object_name, "fr3_rightfinger", false);
        moveit_msgs::msg::PlanningScene ps_msg;
        ps_msg.is_diff = true;

        // scene->getPlanningSceneMsg(ps_msg);

        // planning_scene_interface_->applyPlanningScene(ps_msg);

        // log results
        auto result = std::make_shared<CustomActionOpen::Result>();
        result->success = success;
        if (success) {
            RCLCPP_INFO(this->get_logger(), "Goal reached successfully");
            result->message = "Goal reached successfully";
            goal_handle->succeed(result);
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal failed");
            result->message = "Goal failed";
            goal_handle->abort(result);
        }
        RCLCPP_INFO(this->get_logger(), "\n\n\n\n--------------- OPEN CALLBACK END ---------------\n\n\n\n");
    }).detach();
}

void hts_node::handle_accepted_pickup_(const std::shared_ptr<rclcpp_action::ServerGoalHandle<CustomActionPickup>> goal_handle) {
    std::thread([this, goal_handle] {
        RCLCPP_INFO(this->get_logger(), "\n\n\n\n--------------- PICKUP CALLBACK ---------------\n\n\n\n");

        auto result = std::make_shared<CustomActionPickup::Result>();
        auto feedback = std::make_shared<CustomActionPickup::Feedback>();
        moveit::core::MoveItErrorCode success;

        // set the goal
        geometry_msgs::msg::Pose target = goal_handle->get_goal()->pose;
        this->log_pose(target, "Target Pickup");
        
        planning_scene_monitor_->updateFrameTransforms();
        std::shared_ptr<moveit::core::RobotState> current_state = move_group_interface_->getCurrentState(10.0);
        current_state->enforceBounds();

        if (!current_state) {
            RCLCPP_WARN(this->get_logger(), "Current State is NULL");
        } else {
            RCLCPP_INFO(this->get_logger(), "Current Joint State: ");
            this->print_robot_state(*current_state);
        }

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        success = this->plan_pickup(*current_state, target, plan, DEFAULT_PICKUP_TIME);
        
        if (!success) {
            RCLCPP_ERROR(this->get_logger(), "Planning Failed");
            result->success = false;
            result->message = "Planning failed";
            goal_handle->abort(result);
            return;
        }
            
        feedback->progress = "Planning succeeded. Executing...";
        goal_handle->publish_feedback(feedback);

        RCLCPP_INFO(this->get_logger(), "Current Joint State: ");
        print_robot_state(*current_state);

        RCLCPP_INFO(this->get_logger(), "Plan Start State: ");
        print_robot_state(plan.start_state);

        const auto& pt = plan.trajectory.joint_trajectory.points.front();
        for (size_t i = 0; i < pt.positions.size(); ++i) {
            RCLCPP_INFO(this->get_logger(), "traj start %s: %f",
            plan.trajectory.joint_trajectory.joint_names[i].c_str(),
            pt.positions[i]);
        }

        success = (move_group_interface_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        geometry_msgs::msg::Pose end_pose = move_group_interface_->getCurrentPose().pose;
        this->log_pose(end_pose, "End Pickup");
        
        if (success) {
            RCLCPP_INFO(this->get_logger(), "Plan Execution Succeeded");
            result->success = true;
            result->message = "Planning & execution succeeded";
            goal_handle->succeed(result);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Plan Execution Failed");
            result->success = false;
            result->message = "Plan execution failed";
            goal_handle->abort(result);
        }

        RCLCPP_INFO(this->get_logger(), "\n\n\n\n--------------- PICKUP CALLBACK END ---------------\n\n\n\n");
    }).detach();
}

void hts_node::handle_accepted_move_(const std::shared_ptr<rclcpp_action::ServerGoalHandle<CustomActionMove>> goal_handle) {
    std::thread([this, goal_handle] {
        RCLCPP_INFO(this->get_logger(), "\n\n\n\n--------------- MOVE CALLBACK ---------------\n\n\n\n");

        auto result = std::make_shared<CustomActionMove::Result>();
        auto feedback = std::make_shared<CustomActionMove::Feedback>();
        moveit::core::MoveItErrorCode success;

        auto goal = goal_handle->get_goal();

        std::shared_ptr<moveit::core::RobotState> current_state = move_group_interface_->getCurrentState(10.0);
        current_state->enforceBounds();
        geometry_msgs::msg::Pose current_pose = move_group_interface_->getCurrentPose().pose;

        if (!current_state) {
            RCLCPP_WARN(this->get_logger(), "Current State is NULL");
        }

        geometry_msgs::msg::Pose target_pose;
        target_pose.orientation.x = current_pose.orientation.x;
        target_pose.orientation.y = current_pose.orientation.y;
        target_pose.orientation.z = current_pose.orientation.z;
        target_pose.orientation.w = current_pose.orientation.w;
        target_pose.position.x = goal->x;
        target_pose.position.y = goal->y;
        target_pose.position.z = goal->z;
        
        this->log_pose(target_pose, "Target Move");

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        success = this->plan_move(*current_state, current_pose, target_pose, plan, DEFAULT_MOVE_TIME);
        
        if (!success) {
            RCLCPP_ERROR(this->get_logger(), "Planning Failed");
            result->success = false;
            result->message = "Planning failed";
            goal_handle->abort(result);
            return;
        }

        feedback->progress = "Planning succeeded. Executing...";
        goal_handle->publish_feedback(feedback);

        success = (move_group_interface_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        geometry_msgs::msg::Pose end_pose = move_group_interface_->getCurrentPose().pose;
        log_pose(end_pose, "End Move");
        
        if (success) {
            RCLCPP_INFO(this->get_logger(), "Plan Execution Succeeded");
            result->success = true;
            result->message = "Planning & execution succeeded";
            goal_handle->succeed(result);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Plan Execution Failed");
            result->success = false;
            result->message = "Plan execution failed";
            goal_handle->abort(result);
        }

        RCLCPP_INFO(this->get_logger(), "\n\n\n\n--------------- MOVE CALLBACK END ---------------\n\n\n\n");
    }).detach();
}

rclcpp_action::GoalResponse hts_node::handle_goal_compute_grasp_validity_(const rclcpp_action::GoalUUID&, std::shared_ptr<const CustomActionComputeGraspValidity::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received Compute Grasp Validity Request");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse hts_node::handle_cancel_compute_grasp_validity_(const std::shared_ptr<rclcpp_action::ServerGoalHandle<CustomActionComputeGraspValidity>> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel compute grasp validity");
    return rclcpp_action::CancelResponse::REJECT;
}

rclcpp_action::GoalResponse hts_node::handle_goal_open_(const rclcpp_action::GoalUUID&, std::shared_ptr<const CustomActionOpen::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received Gripper Open Request on Object %d", (int) goal->target_id);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse hts_node::handle_cancel_open_(const std::shared_ptr<rclcpp_action::ServerGoalHandle<CustomActionOpen>> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel gripper open");
    return rclcpp_action::CancelResponse::REJECT;
}

rclcpp_action::GoalResponse hts_node::handle_goal_move_(const rclcpp_action::GoalUUID&, std::shared_ptr<const CustomActionMove::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received move request");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse hts_node::handle_cancel_move_(const std::shared_ptr<rclcpp_action::ServerGoalHandle<CustomActionMove>> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel move");
    return rclcpp_action::CancelResponse::REJECT;
}

rclcpp_action::GoalResponse hts_node::handle_goal_pickup_(const rclcpp_action::GoalUUID&, std::shared_ptr<const CustomActionPickup::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received pickup request Position");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse hts_node::handle_cancel_pickup_(const std::shared_ptr<rclcpp_action::ServerGoalHandle<CustomActionPickup>> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel pickup");
    return rclcpp_action::CancelResponse::REJECT;
}

rclcpp_action::GoalResponse hts_node::handle_goal_close_(const rclcpp_action::GoalUUID&, std::shared_ptr<const CustomActionClose::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received Gripper Close Request on Object %d", (int)goal->target_id);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse hts_node::handle_cancel_close_(const std::shared_ptr<rclcpp_action::ServerGoalHandle<CustomActionClose>> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel gripper close");
    return rclcpp_action::CancelResponse::REJECT;
}