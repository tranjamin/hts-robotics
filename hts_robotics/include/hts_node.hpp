// #include <rclcpp/rclcpp.hpp>

// #include <chrono>
// #include <memory>
// #include <string>

// // includes
// #include <moveit/move_group_interface/move_group_interface.hpp>
// #include <moveit/planning_scene_interface/planning_scene_interface.hpp>
// #include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>

// #include <moveit/planning_interface/planning_interface.hpp>
// #include <moveit/planning_pipeline/planning_pipeline.hpp>

// #include <moveit/robot_state/robot_state.hpp>
// #include <moveit/robot_model/joint_model_group.hpp>

// #include <moveit/collision_detection/collision_common.hpp>

// #include "rclcpp/rclcpp.hpp"
// #include <rclcpp_action/rclcpp_action.hpp>
// #include "std_msgs/msg/string.hpp"

// #include "geometry_msgs/msg/point_stamped.hpp"
// #include "geometry_msgs/msg/pose_stamped.hpp"
// #include "geometry_msgs/msg/point.hpp"
// #include "geometry_msgs/msg/quaternion.hpp"
// #include "sensor_msgs/msg/joint_state.hpp"
// #include "trajectory_msgs/msg/joint_trajectory.hpp"
// #include <tf2_msgs/msg/tf_message.hpp>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_eigen/tf2_eigen.hpp>

// #include "hts_msgs/action/move_target.hpp"
// #include "hts_msgs/action/pick_up_target.hpp"
// #include "hts_msgs/action/gripper_open.hpp"
// #include "hts_msgs/action/gripper_close.hpp"
// #include "hts_msgs/action/grasp_object.hpp"
// #include "hts_msgs/action/compute_grasp_validity.hpp"
// #include "hts_msgs/action/request_grasp.hpp"
// #include "hts_msgs/srv/get_object_position.hpp"

// #include <geometric_shapes/shape_operations.h>
// #include <shape_msgs/msg/mesh.hpp>

// #include <pluginlib/class_loader.hpp>
// #include <hts_plugins/hts_constraint_sampler.hpp>
// #include <moveit/constraint_samplers/constraint_sampler_manager.hpp>

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>

#include <moveit/planning_interface/planning_interface.hpp>
#include <moveit/planning_pipeline/planning_pipeline.hpp>

#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include "hts_msgs/action/move_target.hpp"
#include "hts_msgs/action/pick_up_target.hpp"
#include "hts_msgs/action/gripper_open.hpp"
#include "hts_msgs/action/gripper_close.hpp"
#include "hts_msgs/action/grasp_object.hpp"
#include "hts_msgs/action/compute_grasp_validity.hpp"
#include "hts_msgs/action/request_grasp.hpp"
#include "hts_msgs/srv/get_object_position.hpp"

#include <hts_plugins/hts_constraint_sampler.hpp>
#include <moveit/constraint_samplers/constraint_sampler_manager.hpp>

class hts_node : public rclcpp::Node {
    public:

        // type definitions
        using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
        using PlanningSceneInterface = moveit::planning_interface::PlanningSceneInterface;
        using PlanningSceneMonitor = planning_scene_monitor::PlanningSceneMonitor;

        using CustomActionPickup = hts_msgs::action::PickUpTarget;
        using CustomActionMove = hts_msgs::action::MoveTarget;

        using CustomActionOpen = hts_msgs::action::GripperOpen;
        using CustomActionClose = hts_msgs::action::GripperClose;
        
        using CustomActionComputeGraspValidity = hts_msgs::action::ComputeGraspValidity;

        // constructor
        hts_node(
            const rclcpp::NodeOptions& options = (rclcpp::NodeOptions()
                .allow_undeclared_parameters(true)
                .automatically_declare_parameters_from_overrides(true)
            )
        );

        // initialiser
        void init();

    private:
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Time last_update_;
        std::vector<std::string> target_object_ids_;

        // move groups, planning scenes and planning pipelines
        std::shared_ptr<MoveGroupInterface> move_group_interface_;
        std::shared_ptr<PlanningSceneInterface> planning_scene_interface_;
        std::shared_ptr<MoveGroupInterface> gripper_interface_;
        std::shared_ptr<PlanningSceneMonitor> planning_scene_monitor_;
        std::shared_ptr<planning_pipeline::PlanningPipeline> planning_pipeline;
        std::shared_ptr<planning_pipeline::PlanningPipeline> planning_pipeline_ompl;

        // subscribers and publishers
        rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr gazebo_scene_sub_;
        rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_diff_publisher_;
        
        // action servers and clients
        rclcpp_action::Server<CustomActionPickup>::SharedPtr pickup_server_;
        rclcpp_action::Server<CustomActionMove>::SharedPtr move_server_;
        rclcpp_action::Server<CustomActionOpen>::SharedPtr gripper_open_server_;
        rclcpp_action::Server<CustomActionClose>::SharedPtr gripper_close_server_;
        rclcpp_action::Server<CustomActionComputeGraspValidity>::SharedPtr compute_grasp_validity_server_;

        // for the top-level actions
        rclcpp::Service<hts_msgs::srv::GetObjectPosition>::SharedPtr object_position_service_;

        // sampler manangers
        // const constraint_samplers::ConstraintSamplerAllocatorPtr sampler_allocator;
        std::shared_ptr<constraint_samplers::ConstraintSamplerManager>  sampler_manager;

        // helper and logging functions
        void log_planning_details();
        void log_pose(const geometry_msgs::msg::Pose &pose, const char* descriptor);
        void printCollisionContacts(const collision_detection::CollisionResult& res, const rclcpp::Logger& logger);
        void printConstraints(const moveit_msgs::msg::Constraints& c);
        void printMotionPlanRequestFull(const moveit_msgs::msg::MotionPlanRequest& request);
        void print_robot_state(moveit_msgs::msg::RobotState msg);
        void print_robot_state(moveit::core::RobotState state);
        double compute_trajectory_length_(trajectory_msgs::msg::JointTrajectory trajectory);

        // planning functions
        moveit::core::MoveItErrorCode refine_path_with_stomp(moveit::planning_interface::MoveGroupInterface::Plan &plan);
        bool compute_IK_manually(const geometry_msgs::msg::Pose& target_pose, moveit::core::RobotState& ik_state);
        moveit::core::MoveItErrorCode plan_move(
            const moveit::core::RobotState& start_state, 
            const geometry_msgs::msg::Pose& start_pose, 
            const geometry_msgs::msg::Pose& target_pose,
             moveit::planning_interface::MoveGroupInterface::Plan &plan
        );
        moveit::core::MoveItErrorCode plan_move_no_refine(
            const moveit::core::RobotState& start_state, 
            const geometry_msgs::msg::Pose& start_pose, 
            const geometry_msgs::msg::Pose& target_pose,
             moveit::planning_interface::MoveGroupInterface::Plan &plan
        );
        moveit::core::MoveItErrorCode plan_pickup(
            const moveit::core::RobotState& start_state, 
            const geometry_msgs::msg::Pose& target_pose, 
            moveit::planning_interface::MoveGroupInterface::Plan &plan
        );

        // planning scene utilities
        void gazebo_scene_subscriber_callback_(tf2_msgs::msg::TFMessage::UniquePtr msg);
        void add_ground_collision();
        void load_target_objects();
        void register_grasped_object(std::string object_name);
        void deregister_grasped_object(std::string object_name);

        // action execution handlers
        void handle_accepted_compute_grasp_validity_(const std::shared_ptr<rclcpp_action::ServerGoalHandle<CustomActionComputeGraspValidity>> goal_handle);
        void handle_accepted_close_(const std::shared_ptr<rclcpp_action::ServerGoalHandle<CustomActionClose>> goal_handle);
        void handle_accepted_open_(const std::shared_ptr<rclcpp_action::ServerGoalHandle<CustomActionOpen>> goal_handle);
        void handle_accepted_pickup_(const std::shared_ptr<rclcpp_action::ServerGoalHandle<CustomActionPickup>> goal_handle);
        void handle_accepted_move_(const std::shared_ptr<rclcpp_action::ServerGoalHandle<CustomActionMove>> goal_handle);
        
        // action accept and cancel handlers
        rclcpp_action::GoalResponse handle_goal_compute_grasp_validity_(const rclcpp_action::GoalUUID&, std::shared_ptr<const CustomActionComputeGraspValidity::Goal> goal);
        rclcpp_action::CancelResponse handle_cancel_compute_grasp_validity_(const std::shared_ptr<rclcpp_action::ServerGoalHandle<CustomActionComputeGraspValidity>> goal_handle);
        rclcpp_action::GoalResponse handle_goal_open_(const rclcpp_action::GoalUUID&, std::shared_ptr<const CustomActionOpen::Goal> goal);
        rclcpp_action::CancelResponse handle_cancel_open_(const std::shared_ptr<rclcpp_action::ServerGoalHandle<CustomActionOpen>> goal_handle);
        rclcpp_action::GoalResponse handle_goal_move_(const rclcpp_action::GoalUUID&, std::shared_ptr<const CustomActionMove::Goal> goal);
        rclcpp_action::CancelResponse handle_cancel_move_(const std::shared_ptr<rclcpp_action::ServerGoalHandle<CustomActionMove>> goal_handle);
        rclcpp_action::GoalResponse handle_goal_pickup_(const rclcpp_action::GoalUUID&, std::shared_ptr<const CustomActionPickup::Goal> goal);
        rclcpp_action::CancelResponse handle_cancel_pickup_(const std::shared_ptr<rclcpp_action::ServerGoalHandle<CustomActionPickup>> goal_handle);
        rclcpp_action::GoalResponse handle_goal_close_(const rclcpp_action::GoalUUID&, std::shared_ptr<const CustomActionClose::Goal> goal);
        rclcpp_action::CancelResponse handle_cancel_close_(const std::shared_ptr<rclcpp_action::ServerGoalHandle<CustomActionClose>> goal_handle);

        void get_object_position(const std::shared_ptr<hts_msgs::srv::GetObjectPosition::Request> request, std::shared_ptr<hts_msgs::srv::GetObjectPosition::Response> response);
};