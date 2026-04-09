#include "hts_node.hpp"

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <moveit/planning_pipeline/planning_pipeline.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>

#include <pluginlib/class_loader.hpp>
#include <hts_plugins/hts_constraint_sampler.hpp>
#include <moveit/constraint_samplers/constraint_sampler_manager.hpp>

#include "hts_msgs/action/move_target.hpp"
#include "hts_msgs/action/pick_up_target.hpp"
#include "hts_msgs/action/gripper_open.hpp"
#include "hts_msgs/action/gripper_close.hpp"
#include "hts_msgs/action/grasp_object.hpp"
#include "hts_msgs/action/compute_grasp_validity.hpp"
#include "hts_msgs/action/request_grasp.hpp"
#include "hts_msgs/srv/get_object_position.hpp"


// constructor
hts_node::hts_node(const rclcpp::NodeOptions& options) : Node("hts_node", options) {
	RCLCPP_INFO(this->get_logger(), "Constructing HTS Robotics Node...");

	// this->declare_parameter("stomp_moveit", "");
	// this->declare_parameter("stomp_moveit.planning_pipeline", "stomp_moveit/StompPlanner");
	// this->declare_parameter("test_parameter", "");

	// create a subscriber for the gazebo scene
	RCLCPP_DEBUG(this->get_logger(), "Creating Gazebo scene subscriber...");
	gazebo_scene_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
		"/world/empty/dynamic_pose/info", 1,
		std::bind(&hts_node::gazebo_scene_subscriber_callback_, this, std::placeholders::_1)
	);
	RCLCPP_DEBUG(this->get_logger(), "Created Gazebo scene subscriber.");

	// create pickup server
	RCLCPP_DEBUG(this->get_logger(), "Creating Pickup Server...");
	pickup_server_ = rclcpp_action::create_server<CustomActionPickup>(
		this, "hts_pickup_action",
		std::bind(&hts_node::handle_goal_pickup_, this, std::placeholders::_1, std::placeholders::_2),
		std::bind(&hts_node::handle_cancel_pickup_, this, std::placeholders::_1),
		std::bind(&hts_node::handle_accepted_pickup_, this, std::placeholders::_1)
		// action_options
	);
	RCLCPP_DEBUG(this->get_logger(), "Created Pickup Server.");

	// create move server
	RCLCPP_DEBUG(this->get_logger(), "Creating Move Server...");
	move_server_ = rclcpp_action::create_server<CustomActionMove>(
		this, "hts_move_action",
		std::bind(&hts_node::handle_goal_move_, this, std::placeholders::_1, std::placeholders::_2),
		std::bind(&hts_node::handle_cancel_move_, this, std::placeholders::_1),
		std::bind(&hts_node::handle_accepted_move_, this, std::placeholders::_1)
		// action_options
	);
	RCLCPP_DEBUG(this->get_logger(), "Created Move Server.");

	// create gripper servers
	RCLCPP_DEBUG(this->get_logger(), "Creating Gripper Servers...");
	gripper_open_server_ = rclcpp_action::create_server<CustomActionOpen>(
		this, "gripper_open",
		std::bind(&hts_node::handle_goal_open_, this, std::placeholders::_1, std::placeholders::_2),
		std::bind(&hts_node::handle_cancel_open_, this, std::placeholders::_1),
		std::bind(&hts_node::handle_accepted_open_, this, std::placeholders::_1)
	);
	gripper_close_server_ = rclcpp_action::create_server<CustomActionClose>(
		this, "gripper_close",
		std::bind(&hts_node::handle_goal_close_, this, std::placeholders::_1, std::placeholders::_2),
		std::bind(&hts_node::handle_cancel_close_, this, std::placeholders::_1),
		std::bind(&hts_node::handle_accepted_close_, this, std::placeholders::_1)
	);
	RCLCPP_DEBUG(this->get_logger(), "Created Gripper Servers.");

	// create grasping servers
	RCLCPP_DEBUG(this->get_logger(), "Creating Grasping Validity Servers...");
	compute_grasp_validity_server_ = rclcpp_action::create_server<CustomActionComputeGraspValidity>(
		this, "compute_grasp_validity",
		std::bind(&hts_node::handle_goal_compute_grasp_validity_, this, std::placeholders::_1, std::placeholders::_2),
		std::bind(&hts_node::handle_cancel_compute_grasp_validity_, this, std::placeholders::_1),
		std::bind(&hts_node::handle_accepted_compute_grasp_validity_, this, std::placeholders::_1)
	);
	RCLCPP_DEBUG(this->get_logger(), "Created Grasping Validity Servers.");

	RCLCPP_DEBUG(this->get_logger(), "Creating Object Position Service...");
	object_position_service_ = this->create_service<hts_msgs::srv::GetObjectPosition>(
		"get_object_position",
		std::bind(&hts_node::get_object_position, this, std::placeholders::_1, std::placeholders::_2)
	);
	RCLCPP_DEBUG(this->get_logger(), "Created Object Position Service.");

	RCLCPP_INFO(this->get_logger(), "Finished constructing HTS Node.");
}

// initialise
void hts_node::init() {
    RCLCPP_INFO(this->get_logger(), "Starting HTS node initialisation...");

    last_update_ = this->now();

    move_group_interface_ = std::make_shared<MoveGroupInterface>(shared_from_this(), "fr3_arm");
    gripper_interface_ = std::make_shared<MoveGroupInterface>(shared_from_this(), "fr3_hand");
    planning_scene_interface_ = std::make_shared<PlanningSceneInterface>();
    planning_scene_monitor_ = std::make_shared<PlanningSceneMonitor>(shared_from_this(), "robot_description");
    RCLCPP_DEBUG(this->get_logger(), "Initialised move groups and planning scenes.");

    // start planning scene monitor
    planning_scene_monitor_->startSceneMonitor();
    planning_scene_monitor_->startWorldGeometryMonitor();
    planning_scene_monitor_->startStateMonitor();
    RCLCPP_DEBUG(this->get_logger(), "Started scene monitors.");

    RCLCPP_INFO(this->get_logger(), "Making Sampler Allocator...");
    const constraint_samplers::ConstraintSamplerAllocatorPtr sampler_allocator = std::make_shared<hts_plugins::HTSIKConstraintSamplerAllocator>();
    // auto sampler = sampler_allocator->alloc(scene, "fr3_arm", constraints)
    sampler_manager = std::make_shared<constraint_samplers::ConstraintSamplerManager>();
    sampler_manager->registerSamplerAllocator(sampler_allocator);
    RCLCPP_INFO(this->get_logger(), "Made Sampler Allocator...");

    pluginlib::ClassLoader<constraint_samplers::ConstraintSamplerAllocator> loader(
        "moveit_core", 
        "constraint_samplers::ConstraintSamplerAllocator"
    );
    std::vector<std::string> classes = loader.getDeclaredClasses();
    for (const auto &c : classes)
      RCLCPP_INFO(this->get_logger(), "Found a plugin class: %s", c.c_str());

    RCLCPP_INFO(this->get_logger(), "Made Sampler Allocator...");

    // moveit_msgs::msg::Constraints constraints;
    // auto allocator = manager->getAllocator("fr3_arm", constraints);
    // auto sampler = allocator->alloc(scene, "fr3_arm", constraints);

    planning_pipeline = std::make_shared<planning_pipeline::PlanningPipeline>(
      planning_scene_monitor_->getRobotModel(),
      shared_from_this(),
      "stomp_moveit"
    );
    planning_pipeline_ompl = std::make_shared<planning_pipeline::PlanningPipeline>(
      planning_scene_monitor_->getRobotModel(),
      shared_from_this(),
      "ompl"
    );

    // set tolerances for gripper
    gripper_interface_->setGoalPositionTolerance(0.001);
    gripper_interface_->setGoalJointTolerance(0.001);
    gripper_interface_->setGoalOrientationTolerance(0.1);    
    gripper_interface_->setWorkspace(-2.0, -2.0, 0.0, 2.0, 2.0, 2.0);
    gripper_interface_->setPlanningPipelineId("ompl");
    gripper_interface_->setPlannerId("ompl");

    // set tolerances for arm
    move_group_interface_->setGoalPositionTolerance(0.002);
    move_group_interface_->setGoalOrientationTolerance(0.01);
    move_group_interface_->setGoalJointTolerance(0.01);
    move_group_interface_->setPlanningTime(20.0);
    move_group_interface_->setWorkspace(-2.0, -2.0, 0.0, 2.0, 2.0, 2.0);
    move_group_interface_->setMaxVelocityScalingFactor(0.5);
    move_group_interface_->setMaxAccelerationScalingFactor(0.3);
    move_group_interface_->setPlanningPipelineId("ompl");
    move_group_interface_->setPlannerId("fr3_arm[RRTConnectkConfigDefault]");

    this->log_planning_details();

    RCLCPP_DEBUG(this->get_logger(), "Set planning tolerances.");

    // for dynamically updating the planning scene (enabling and disabling collisions)
    planning_scene_diff_publisher_ = this->create_publisher<moveit_msgs::msg::PlanningScene>("/planning_scene", 10);

    this->add_ground_collision();
    this->load_target_objects();
}