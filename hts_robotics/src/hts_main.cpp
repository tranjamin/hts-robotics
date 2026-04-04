#include <chrono>
#include <memory>
#include <string>

// includes
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>

#include <moveit/planning_interface/planning_interface.hpp>
#include <moveit/planning_pipeline/planning_pipeline.hpp>

#include <moveit/robot_state/robot_state.hpp>
#include <moveit/robot_model/joint_model_group.hpp>

#include <moveit/collision_detection/collision_common.hpp>

#include "rclcpp/rclcpp.hpp"
#include <rclcpp_action/rclcpp_action.hpp>
#include "std_msgs/msg/string.hpp"

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include "hts_msgs/action/move_target.hpp"
#include "hts_msgs/action/pick_up_target.hpp"
#include "hts_msgs/action/gripper_open.hpp"
#include "hts_msgs/action/gripper_close.hpp"
#include "hts_msgs/action/grasp_object.hpp"
#include "hts_msgs/action/compute_grasp_validity.hpp"
#include "hts_msgs/action/request_grasp.hpp"
#include "hts_msgs/srv/get_object_position.hpp"

#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/msg/mesh.hpp>

#include <pluginlib/class_loader.hpp>
#include <hts_plugins/hts_constraint_sampler.hpp>
#include <moveit/constraint_samplers/constraint_sampler_manager.hpp>

// for using macros like s, ms, us
using namespace std::chrono_literals;

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
  ):Node("hts_node", options) {
    RCLCPP_INFO(this->get_logger(), "Constructing HTS Robotics Node...");

    // this->declare_parameter("stomp_moveit", "");
    // this->declare_parameter("stomp_moveit.planning_pipeline", "stomp_moveit/StompPlanner");
    // this->declare_parameter("test_parameter", "");

    action_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    sub_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    // action_options_.callback_group = action_callback_group_;

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
      // action_options
    );
    gripper_close_server_ = rclcpp_action::create_server<CustomActionClose>(
      this, "gripper_close",
      std::bind(&hts_node::handle_goal_close_, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&hts_node::handle_cancel_close_, this, std::placeholders::_1),
      std::bind(&hts_node::handle_accepted_close_, this, std::placeholders::_1)
      // action_options
    );
    RCLCPP_DEBUG(this->get_logger(), "Created Gripper Servers.");

    // create grasping servers
    RCLCPP_DEBUG(this->get_logger(), "Creating Grasping Validity Servers...");
    compute_grasp_validity_server_ = rclcpp_action::create_server<CustomActionComputeGraspValidity>(
      this, "compute_grasp_validity",
      std::bind(&hts_node::handle_goal_compute_grasp_validity_, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&hts_node::handle_cancel_compute_grasp_validity_, this, std::placeholders::_1),
      std::bind(&hts_node::handle_accepted_compute_grasp_validity_, this, std::placeholders::_1)
      // action_options
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
  void init() {
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

    log_planning_details();

    RCLCPP_DEBUG(this->get_logger(), "Set planning tolerances.");

    // for dynamically updating the planning scene (enabling and disabling collisions)
    planning_scene_diff_publisher_ = this->create_publisher<moveit_msgs::msg::PlanningScene>("/planning_scene", 10);

    // Register the ground as a collision object
    moveit_msgs::msg::CollisionObject co_ground;
    shape_msgs::msg::SolidPrimitive primitive_ground;
    geometry_msgs::msg::Pose pose_ground;

    primitive_ground.type = primitive_ground.BOX;
    primitive_ground.dimensions = {3, 3, 0.5};

    pose_ground.orientation.w = 1.0;
    pose_ground.position.x = 0;
    pose_ground.position.y = 0;
    pose_ground.position.z = -0.251;

    co_ground.id = "ground";
    co_ground.header.frame_id = move_group_interface_->getPlanningFrame();
    co_ground.primitives.push_back(primitive_ground);
    co_ground.primitive_poses.push_back(pose_ground);
    co_ground.operation = co_ground.ADD;
    planning_scene_interface_->applyCollisionObject(co_ground);
    RCLCPP_INFO(get_logger(), "Applied collision object 'ground' to planning scene.");

    load_target_objects();
  }

  private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_update_;
    std::vector<std::string> target_object_ids_;

    // move groups and planning scenes
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

    rclcpp::CallbackGroup::SharedPtr action_callback_group_;
    rclcpp::CallbackGroup::SharedPtr sub_callback_group_;
    // rclcpp_action::ServerOptions action_options_;

    // const constraint_samplers::ConstraintSamplerAllocatorPtr sampler_allocator;
    std::shared_ptr<constraint_samplers::ConstraintSamplerManager>  sampler_manager;

    void log_planning_details() {
      std::vector<moveit_msgs::msg::PlannerInterfaceDescription> desc;
      move_group_interface_->getInterfaceDescriptions(desc);

      RCLCPP_INFO(this->get_logger(), "Loaded planning pipelines and planners:");
      for (const auto &pipeline : desc)
      {
          RCLCPP_INFO(this->get_logger(), "Pipeline name: %s", pipeline.name.c_str());
          for (const auto &planner_id : pipeline.planner_ids)
          {
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
      for (const auto &planner_id : default_desc.planner_ids)
        {
          RCLCPP_INFO(this->get_logger(), "  Planner ID: %s", planner_id.c_str());
        }

      RCLCPP_INFO(this->get_logger(), "default planning pipeline id: %s", move_group_interface_->getDefaultPlanningPipelineId().c_str());
      RCLCPP_INFO(this->get_logger(), "default planner id: %s", move_group_interface_->getDefaultPlannerId().c_str());
      RCLCPP_INFO(this->get_logger(), "current planner id: %s", move_group_interface_->getPlannerId().c_str());
    }

    void load_target_objects() {
      // Register the target objects as collision objects
      // this->declare_parameter("objects", std::vector<std::string>{});
      std::vector<std::string> objects = this->get_parameter("objects").as_string_array();

      target_object_ids_ = std::vector<std::string>(objects.size(), "");
      int i = 0;

      RCLCPP_DEBUG(get_logger(), "Looking for objects: %ld", objects.size());
      for (auto &obj_name : objects) {
        moveit_msgs::msg::CollisionObject co_target;
        shape_msgs::msg::SolidPrimitive primitive_target;
        shape_msgs::msg::Mesh mesh_target;
        geometry_msgs::msg::Pose pose_target;

        // this->declare_parameter(obj_name + ".object_id", 0);
        // this->declare_parameter(obj_name + ".primitive_type", "");

        int obj_id = this->get_parameter(obj_name + ".object_id").as_int();
        RCLCPP_DEBUG(get_logger(), "Found an Object with ID %d", obj_id);

        // this->declare_parameter(obj_name + ".x", 0.0);
        // this->declare_parameter(obj_name + ".y", 0.0);
        // this->declare_parameter(obj_name + ".z", 0.0);

        // this->declare_parameter(obj_name + ".R", 0.0);
        // this->declare_parameter(obj_name + ".P", 0.0);
        // this->declare_parameter(obj_name + ".Y", 0.0);

        pose_target.position.x = this->get_parameter(obj_name + ".x").as_double();
        pose_target.position.y = this->get_parameter(obj_name + ".y").as_double();
        pose_target.position.z = this->get_parameter(obj_name + ".z").as_double();

        // double roll = this->get_parameter(obj_name + ".R").as_double();
        // double pitch = this->get_parameter(obj_name + ".P").as_double();
        // double yaw = this->get_parameter(obj_name + ".Y").as_double();
        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 0.0;

        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);

        pose_target.orientation.x = q.x();
        pose_target.orientation.y = q.y();
        pose_target.orientation.z = q.z();
        pose_target.orientation.w = q.w();
        
        co_target.id = "target_" + std::to_string(obj_id);
        co_target.header.frame_id = move_group_interface_->getPlanningFrame();

        std::string obj_type = this->get_parameter(obj_name + ".primitive_type").as_string();
        if (obj_type == "BOX") {
          // this->declare_parameter(obj_name + ".primitive_dims.x", 1.0);
          // this->declare_parameter(obj_name + ".primitive_dims.y", 1.0);
          // this->declare_parameter(obj_name + ".primitive_dims.z", 1.0);

          primitive_target.type = primitive_target.BOX;
          primitive_target.dimensions = {
            this->get_parameter(obj_name + ".primitive_dims.x").as_double(),
            this->get_parameter(obj_name + ".primitive_dims.y").as_double(),
            this->get_parameter(obj_name + ".primitive_dims.z").as_double()
          };

          co_target.primitives.push_back(primitive_target);
          co_target.primitive_poses.push_back(pose_target);
        } else if (obj_type == "SPHERE") {
          // this->declare_parameter(obj_name + ".primitive_dims.radius", 1.0);

          primitive_target.type = primitive_target.SPHERE;
          primitive_target.dimensions = {
            this->get_parameter(obj_name + ".primitive_dims.radius").as_double()
          };

          co_target.primitives.push_back(primitive_target);
          co_target.primitive_poses.push_back(pose_target);

        } else if (obj_type == "CONE") {        
          // this->declare_parameter(obj_name + ".primitive_dims.height", 1.0);
          // this->declare_parameter(obj_name + ".primitive_dims.radius", 1.0);

          primitive_target.type = primitive_target.CONE;
          primitive_target.dimensions = {
            this->get_parameter(obj_name + ".primitive_dims.height").as_double(),
            this->get_parameter(obj_name + ".primitive_dims.radius").as_double()
          };

          co_target.primitives.push_back(primitive_target);
          co_target.primitive_poses.push_back(pose_target);

        } else if (obj_type == "CYLINDER") {
          // this->declare_parameter(obj_name + ".primitive_dims.height", 1.0);
          // this->declare_parameter(obj_name + ".primitive_dims.radius", 1.0);

          primitive_target.type = primitive_target.CYLINDER;
          primitive_target.dimensions = {
            this->get_parameter(obj_name + ".primitive_dims.height").as_double(),
            this->get_parameter(obj_name + ".primitive_dims.radius").as_double()
          };

          co_target.primitives.push_back(primitive_target);
          co_target.primitive_poses.push_back(pose_target);

        } 
        else if (obj_type == "MESH") {
          // this->declare_parameter(obj_name + ".primitive_dims.file", "");
          shapes::Mesh* mesh = shapes::createMeshFromResource( this->get_parameter(obj_name + ".primitive_dims.file").as_string(), Eigen::Vector3d(0.001, 0.001, 0.001));
          shape_msgs::msg::Mesh mesh_msg;
          shapes::ShapeMsg mesh_tmp;
          shapes::constructMsgFromShape(mesh, mesh_tmp);
          mesh_msg = boost::get<shape_msgs::msg::Mesh>(mesh_tmp);

          co_target.meshes.push_back(mesh_msg);
          co_target.mesh_poses.push_back(pose_target);
        } else {
          RCLCPP_ERROR(get_logger(), "Invalid Object");
        }

        co_target.operation = co_target.ADD;
        planning_scene_interface_->applyCollisionObject(co_target);

        target_object_ids_[i] = co_target.id;
      }

      planning_scene_monitor_->requestPlanningSceneState();
    }

    void log_pose(const geometry_msgs::msg::Pose &pose, const char* descriptor="") {
        RCLCPP_INFO(this->get_logger(), "%s Position is (%.2f, %.2f, %.2f)", descriptor, pose.position.x, pose.position.y, pose.position.z);
        RCLCPP_INFO(this->get_logger(), "%s Quaternion is (%.2f, %.2f, %.2f, %.2f)", descriptor, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    }

    bool compute_IK_manually(const geometry_msgs::msg::Pose& target_pose, moveit::core::RobotState& ik_state) {
      std::string group_name = move_group_interface_->getName();
      const moveit::core::JointModelGroup* joint_model_group = planning_scene_monitor_->getRobotModel()->getJointModelGroup(group_name);

      return ik_state.setFromIK(joint_model_group, target_pose, move_group_interface_->getEndEffectorLink(), 1.0);
    }

    moveit::core::MoveItErrorCode refine_path_with_stomp(moveit::planning_interface::MoveGroupInterface::Plan &plan) {
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

    moveit::core::MoveItErrorCode plan_pickup(const moveit::core::RobotState& start_state, const geometry_msgs::msg::Pose& target_pose, moveit::planning_interface::MoveGroupInterface::Plan &plan) {
      move_group_interface_->getCurrentState(10.0);
      move_group_interface_->setStartState(start_state);

      auto current_pose = move_group_interface_->getCurrentPose();
      move_group_interface_->clearPathConstraints();
      move_group_interface_->setPoseTarget(target_pose);

      RCLCPP_INFO(this->get_logger(), "Computing Path using OMPL...");
      moveit::core::MoveItErrorCode ompl_status = move_group_interface_->plan(plan);
      RCLCPP_INFO(this->get_logger(), "OMPL finished with error code %d", ompl_status.val);

      if (ompl_status == moveit::core::MoveItErrorCode::SUCCESS) {
        float trajectory_length_pickup = (float) compute_trajectory_length_(plan.trajectory.joint_trajectory);
        RCLCPP_INFO(this->get_logger(), "Unrefined length is %.5f", trajectory_length_pickup);
      } else {
        return ompl_status;
      }

      print_robot_state(plan.start_state);
      // const auto& pt = plan.trajectory.joint_trajectory.points.front();
      // for (size_t i = 0; i < pt.positions.size(); ++i) {
      //   RCLCPP_INFO(this->get_logger(), "traj start %s: %f",
      //     plan.trajectory.joint_trajectory.joint_names[i].c_str(),
      //     pt.positions[i]);
      // }

      RCLCPP_INFO(this->get_logger(), "Planned from OMPL. Now refining with STOMP");

      moveit::core::MoveItErrorCode stomp_status = refine_path_with_stomp(plan);

      if (stomp_status == moveit::core::MoveItErrorCode::SUCCESS) {
        float trajectory_length_pickup = (float) compute_trajectory_length_(plan.trajectory.joint_trajectory);
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

    moveit::core::MoveItErrorCode plan_move(const moveit::core::RobotState& start_state, const geometry_msgs::msg::Pose& start_pose, const geometry_msgs::msg::Pose& target_pose, moveit::planning_interface::MoveGroupInterface::Plan &plan) {
      move_group_interface_->setStartState(start_state);
      
      // Apply orientation constraints
      moveit_msgs::msg::OrientationConstraint orientation_constraint;
      orientation_constraint.header.frame_id = move_group_interface_->getPoseReferenceFrame();
      orientation_constraint.link_name = move_group_interface_->getEndEffectorLink();
      orientation_constraint.orientation = start_pose.orientation;
      orientation_constraint.absolute_x_axis_tolerance = 0.1;
      orientation_constraint.absolute_y_axis_tolerance = 0.1;
      orientation_constraint.absolute_z_axis_tolerance = 3.042;
      orientation_constraint.weight = 1.0;
      orientation_constraint.parameterization = orientation_constraint.ROTATION_VECTOR;
      
      moveit_msgs::msg::Constraints all_constraints;
      all_constraints.orientation_constraints.emplace_back(orientation_constraint);

      move_group_interface_->clearPathConstraints();
      move_group_interface_->setPathConstraints(all_constraints);
      RCLCPP_INFO(get_logger(), "Applied orientation constraints to planning scene.");

      log_pose(start_pose, "Start Pose");
      log_pose(target_pose, "Target Pose");

      // move_group_interface_->setPositionTarget(target_pose.position.x, target_pose.position.y, target_pose.position.z);
      move_group_interface_->setPoseTarget(target_pose);
      double orig_goal_tolerance = move_group_interface_->getGoalOrientationTolerance();

      RCLCPP_INFO(this->get_logger(), "Displaying some info about the move...");
      planning_interface::MotionPlanResponse motion_plan_response;
      planning_interface::MotionPlanRequest motion_plan_request;
      bool ompl_status;
      moveit::core::MoveItErrorCode err_code;

      #define USE_SELF_PIPELINE true

      if (!USE_SELF_PIPELINE) {
        RCLCPP_INFO(this->get_logger(), "Computing Path using OMPL built in pipeline...");
        ompl_status = (err_code = move_group_interface_->plan(plan)) == moveit::core::MoveItErrorCode::SUCCESS;
      } else {
        move_group_interface_->constructMotionPlanRequest(motion_plan_request);
        motion_plan_request.goal_constraints[0].orientation_constraints[0].absolute_x_axis_tolerance = 0.1;
        motion_plan_request.goal_constraints[0].orientation_constraints[0].absolute_y_axis_tolerance = 0.1;      
        motion_plan_request.goal_constraints[0].orientation_constraints[0].absolute_z_axis_tolerance = 3.142;     
        printMotionPlanRequestFull(motion_plan_request);

        RCLCPP_INFO(this->get_logger(), "Trying to select the constraint sampler...");
        auto scene = planning_scene_monitor_->getPlanningScene();
        moveit_msgs::msg::Constraints constr = motion_plan_request.goal_constraints[0];
        constraint_samplers::ConstraintSamplerPtr chosen_sampler = sampler_manager->selectSampler(scene, "fr3_arm", constr);
        RCLCPP_INFO(this->get_logger(), "Selected the sampler %s.", chosen_sampler->getName().c_str());

        planning_scene_monitor::LockedPlanningSceneRO locked_scene(planning_scene_monitor_);
        RCLCPP_INFO(this->get_logger(), "Computing Path using OMPL self pipeline...");
        ompl_status = planning_pipeline_ompl->generatePlan(locked_scene, motion_plan_request, motion_plan_response);

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

      if (ompl_status == moveit::core::MoveItErrorCode::SUCCESS) {
        float trajectory_length_pickup = (float) compute_trajectory_length_(plan.trajectory.joint_trajectory);
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

      move_group_interface_->clearPathConstraints();
      move_group_interface_->setGoalOrientationTolerance(orig_goal_tolerance);

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

    void get_object_position(
      const std::shared_ptr<hts_msgs::srv::GetObjectPosition::Request> request,
      std::shared_ptr<hts_msgs::srv::GetObjectPosition::Response> response
      ) {
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

  void printCollisionContacts(const collision_detection::CollisionResult& res, const rclcpp::Logger& logger)
    {
      if (!res.collision)
      {
        RCLCPP_INFO(logger, "No collision detected.");
        return;
      }

      RCLCPP_WARN(logger, "Collision detected! Number of contact pairs: %zu", res.contacts.size());

      for (const auto& contact_pair : res.contacts)
      {
        const std::string& body_1 = contact_pair.first.first;
        const std::string& body_2 = contact_pair.first.second;

        const std::vector<collision_detection::Contact>& contacts = contact_pair.second;

        RCLCPP_WARN(logger, "Collision between: [%s] and [%s] (%zu contact points)",
                    body_1.c_str(), body_2.c_str(), contacts.size());

        for (size_t i = 0; i < contacts.size(); ++i)
        {
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

void printConstraints(const moveit_msgs::msg::Constraints& c)
{
    // --- Position Constraints ---
    for (size_t i = 0; i < c.position_constraints.size(); ++i)
    {
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
    for (size_t i = 0; i < c.orientation_constraints.size(); ++i)
    {
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
    for (size_t i = 0; i < c.joint_constraints.size(); ++i)
    {
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

void printMotionPlanRequestFull(const moveit_msgs::msg::MotionPlanRequest& request)
{
    RCLCPP_INFO(this->get_logger(), "=== MotionPlanRequest ===");

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

    RCLCPP_INFO(this->get_logger(), "=========================");
}

    void handle_accepted_compute_grasp_validity_(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<CustomActionComputeGraspValidity>> goal_handle
    ) {
      std::thread([this, goal_handle] {
      
      auto object_name = "target_" + std::to_string(goal_handle->get_goal()->target_id);
      auto result = std::make_shared<CustomActionComputeGraspValidity::Result>();
      moveit::core::MoveItErrorCode err_code;

      planning_scene_monitor::LockedPlanningSceneRO planning_scene(planning_scene_monitor_);

      std::shared_ptr<moveit::core::RobotState> current_state = move_group_interface_->getCurrentState(10.0);
      current_state->enforceBounds();
      move_group_interface_->clearPathConstraints();
      
      geometry_msgs::msg::Pose grasp_pose = goal_handle->get_goal()->grasp_pose;

      RCLCPP_INFO(this->get_logger(), "Computing IK (Pickup)...");
      moveit::core::RobotState computed_ik_pickup(planning_scene->getRobotModel());
      if (!compute_IK_manually(grasp_pose, computed_ik_pickup)) {
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

      // RCLCPP_INFO(this->get_logger(), "Computing Goal Collision (Pickup)...");
      // collision_detection::CollisionRequest collision_request;
      // collision_detection::CollisionResult collision_response;
      // collision_request.contacts = true;
      // collision_request.verbose = true;
      // collision_request.distance = true;
      // collision_request.detailed_distance = true;
      // planning_scene->checkCollision(collision_request, collision_response, computed_ik_pickup);

      // if (collision_response.collision) {
      //   RCLCPP_ERROR(this->get_logger(), "Goal Collision Failed...");
      //   printCollisionContacts(collision_response, get_logger());
      //   result->success = true;
      //   result->is_valid = false;
      //   result->score = 0.0;
      //   result->message = "Plan (pickup goal collisions) is not valid";
      //   result->err_code = moveit::core::MoveItErrorCode::GOAL_IN_COLLISION;
      //   result->err_source = "pickup goal collisions";
      //   result->err_message = "";
      //   goal_handle->succeed(result);
      //   return;
      // } else {
      //   RCLCPP_INFO(this->get_logger(), "Goal Collision Succeeded. Closest distance is %f", collision_response.distance);
      // }

      moveit::planning_interface::MoveGroupInterface::Plan pickup_plan;
      err_code = plan_pickup(*current_state, grasp_pose, pickup_plan);

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

      trajectory_msgs::msg::JointTrajectory pickup_joint_trajectory = pickup_plan.trajectory.joint_trajectory;
      float trajectory_length_pickup = (float) compute_trajectory_length_(pickup_joint_trajectory);
      RCLCPP_INFO(this->get_logger(), "Trajectory length (pickup) is %.5f", trajectory_length_pickup);
      RCLCPP_INFO(this->get_logger(), "\n---\n");
        
      moveit::core::RobotState move_start_state(*current_state);
      const moveit::core::JointModelGroup* joint_model_group = move_start_state.getJointModelGroup(move_group_interface_->getName());
      move_start_state.setJointGroupPositions(joint_model_group, pickup_joint_trajectory.points.back().positions);
      move_start_state.update();
      
      geometry_msgs::msg::Pose actual_grasp_pose = tf2::toMsg(move_start_state.getGlobalLinkTransform(move_group_interface_->getEndEffectorLink()));
      log_pose(actual_grasp_pose, "Check: Path is grasping at this pose");
      log_pose(grasp_pose, "It is meant to grasp at this pose");

      geometry_msgs::msg::Pose goal_pose;
      goal_pose.position.x = goal_handle->get_goal()->goal_x;
      goal_pose.position.y = goal_handle->get_goal()->goal_y;
      goal_pose.position.z = goal_handle->get_goal()->goal_z;
      goal_pose.orientation.x = actual_grasp_pose.orientation.x;
      goal_pose.orientation.y = actual_grasp_pose.orientation.y;
      goal_pose.orientation.z = actual_grasp_pose.orientation.z;
      goal_pose.orientation.w = actual_grasp_pose.orientation.w;

      RCLCPP_INFO(this->get_logger(), "Computing IK (Move)...");
      moveit::core::RobotState computed_ik_move(planning_scene->getRobotModel());
      if (!compute_IK_manually(goal_pose, computed_ik_move)) {
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

      // RCLCPP_INFO(this->get_logger(), "Computing Goal Collision (Pickup)...");
      // planning_scene->checkCollision(collision_request, collision_response, computed_ik_move);

      // if (collision_response.collision) {
      //   RCLCPP_ERROR(this->get_logger(), "Goal Collision Failed...");
      //   result->success = true;
      //   result->is_valid = false;
      //   result->score = 0.0;
      //   result->message = "Plan (move collisions) is not valid";
      //   result->err_code = moveit::core::MoveItErrorCode::NO_IK_SOLUTION;
      //   result->err_source = "move collisions";
      //   result->err_message = "";
      //   goal_handle->succeed(result);
      //   return;
      // } else {
      //   RCLCPP_INFO(this->get_logger(), "Goal Collision Succeeded. Closest distance is %f", collision_response.distance);
      // }

      if (goal_handle->get_goal()->target_id >= 0) {
          // register_grasped_object(object_name);
          gripper_interface_->attachObject(object_name);
      }

      moveit::planning_interface::MoveGroupInterface::Plan move_plan;
      err_code = plan_move(move_start_state, grasp_pose, goal_pose, move_plan);

      if (goal_handle->get_goal()->target_id >= 0) {
        gripper_interface_->detachObject(object_name);
        // deregister_grasped_object(object_name);
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
      trajectory_msgs::msg::JointTrajectory move_joint_trajectory = move_plan.trajectory.joint_trajectory;
      float trajectory_length_move = (float) compute_trajectory_length_(move_joint_trajectory);
      RCLCPP_INFO(this->get_logger(), "Trajectory length (move) is %.5f", trajectory_length_move);

      result->success = true;
      result->is_valid = true;
      result->score = trajectory_length_pickup + trajectory_length_move;
      result->message = "Plan is valid";
      goal_handle->succeed(result);

      RCLCPP_INFO(this->get_logger(), "\n---BREAK---\n");

      }).detach();

    }
    
    double compute_trajectory_length_(trajectory_msgs::msg::JointTrajectory trajectory) {
      double total_length = 0;
      for (size_t i=1; i < trajectory.points.size(); i++) {
        const std::vector<double> &prev_joints = trajectory.points[i - 1].positions;
        const std::vector<double> &curr_joints = trajectory.points[i].positions;
        for (size_t j = 0; j < prev_joints.size(); ++j)
          total_length += std::sqrt(pow(curr_joints[j] - prev_joints[j], 2)); // L1 norm, or use sqrt(sum squared) for L2
      }

      return total_length;
    }

    void register_grasped_object(std::string object_name) {
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
    }

    void deregister_grasped_object(std::string object_name) {
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
    }

    void handle_accepted_close_(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<CustomActionClose>> goal_handle
    ) {
      std::thread([this, goal_handle] {
        RCLCPP_INFO(get_logger(), "\n\n\n\n--------------- CLOSE CALLBACK ---------------\n\n\n\n");

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

        RCLCPP_INFO(get_logger(), "\n\n\n\n--------------- CLOSE CALLBACK END ---------------\n\n\n\n");
      }).detach();
    }

    void handle_accepted_open_(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<CustomActionOpen>> goal_handle
    ) {
      std::thread([this, goal_handle] {
        RCLCPP_INFO(get_logger(), "\n\n\n\n--------------- OPEN CALLBACK ---------------\n\n\n\n");
        
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
        RCLCPP_INFO(get_logger(), "\n\n\n\n--------------- OPEN CALLBACK END ---------------\n\n\n\n");
      }).detach();
    }

    void print_robot_state(moveit_msgs::msg::RobotState msg) {
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

    void print_robot_state(moveit::core::RobotState state) {
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

    void handle_accepted_pickup_(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<CustomActionPickup>> goal_handle
    ) {
      std::thread([this, goal_handle] {
        RCLCPP_INFO(get_logger(), "\n\n\n\n--------------- PICKUP CALLBACK ---------------\n\n\n\n");

        auto result = std::make_shared<CustomActionPickup::Result>();
        auto feedback = std::make_shared<CustomActionPickup::Feedback>();
        moveit::core::MoveItErrorCode success;

        // set the goal
        geometry_msgs::msg::Pose target = goal_handle->get_goal()->pose;
        log_pose(target, "Target Pickup");
        
        planning_scene_monitor_->updateFrameTransforms();
        std::shared_ptr<moveit::core::RobotState> current_state = move_group_interface_->getCurrentState(10.0);
        current_state->enforceBounds();

        if (!current_state) {
          RCLCPP_WARN(this->get_logger(), "Current State is NULL");
        } else {
          RCLCPP_INFO(this->get_logger(), "Current Joint State: ");
          print_robot_state(*current_state);
        }

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        success = plan_pickup(*current_state, target, plan);
      
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
        log_pose(end_pose, "End Pickup");
        
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

        RCLCPP_INFO(get_logger(), "\n\n\n\n--------------- PICKUP CALLBACK END ---------------\n\n\n\n");
      }).detach();
    }

    void handle_accepted_move_(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<CustomActionMove>> goal_handle
    ) {
      std::thread([this, goal_handle] {
        RCLCPP_INFO(get_logger(), "\n\n\n\n--------------- MOVE CALLBACK ---------------\n\n\n\n");

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
        log_pose(target_pose, "Target Move");

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        success = this->plan_move(*current_state, current_pose, target_pose, plan);
      
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

        RCLCPP_INFO(get_logger(), "\n\n\n\n--------------- MOVE CALLBACK END ---------------\n\n\n\n");
      }).detach();
    }

    void gazebo_scene_subscriber_callback_(tf2_msgs::msg::TFMessage::UniquePtr msg) {

    // std::thread([this, msg] {
      rclcpp::Time now = this->now();
      if ((now - last_update_).seconds() < 1) {
        return;
      }
      last_update_ = now;

      for (const auto &obj : msg->transforms) {
          // get current pose
          auto map = planning_scene_interface_->getObjectPoses({obj.child_frame_id});
          if (map.empty()) {
            continue;
          }
          geometry_msgs::msg::Pose target_moveit = map.at(obj.child_frame_id);

          moveit_msgs::msg::CollisionObject target_gazebo;
          target_gazebo.id = obj.child_frame_id;
          target_gazebo.header.frame_id = "world";

          geometry_msgs::msg::Pose gazebo_pose;
          gazebo_pose.orientation.x = obj.transform.rotation.x;
          gazebo_pose.orientation.y = obj.transform.rotation.y;
          gazebo_pose.orientation.z = obj.transform.rotation.z;
          gazebo_pose.orientation.w = obj.transform.rotation.w;
          gazebo_pose.position.x = obj.transform.translation.x;
          gazebo_pose.position.y = obj.transform.translation.y;
          gazebo_pose.position.z = obj.transform.translation.z;

          // exit early if no change
          if (
            gazebo_pose.position.x == target_moveit.position.x &&
            gazebo_pose.position.y == target_moveit.position.y &&
            gazebo_pose.position.z == target_moveit.position.z &&
            gazebo_pose.orientation.x == target_moveit.orientation.x &&
            gazebo_pose.orientation.y == target_moveit.orientation.y &&
            gazebo_pose.orientation.z == target_moveit.orientation.z &&
            gazebo_pose.orientation.w == target_moveit.orientation.w
          ) continue;


          // move target object
          moveit_msgs::msg::CollisionObject co_target;
          co_target.id = obj.child_frame_id;
          co_target.header.frame_id = move_group_interface_->getPlanningFrame();

          geometry_msgs::msg::Pose pose_target;
          pose_target.position.x = gazebo_pose.position.x;
          pose_target.position.y = gazebo_pose.position.y;
          pose_target.position.z = gazebo_pose.position.z;
          pose_target.orientation.x = gazebo_pose.orientation.x;
          pose_target.orientation.y = gazebo_pose.orientation.y;
          pose_target.orientation.z = gazebo_pose.orientation.z;
          pose_target.orientation.w = gazebo_pose.orientation.w;

          co_target.operation = co_target.MOVE;
          co_target.pose = pose_target;
          planning_scene_interface_->applyCollisionObject(co_target);
        }
      // }).detach();
    }

    rclcpp_action::GoalResponse handle_goal_compute_grasp_validity_(
        const rclcpp_action::GoalUUID&, std::shared_ptr<const CustomActionComputeGraspValidity::Goal> goal
      ) {
        RCLCPP_INFO(this->get_logger(), "Received Compute Grasp Validity Request");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      }

    rclcpp_action::CancelResponse handle_cancel_compute_grasp_validity_(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<CustomActionComputeGraspValidity>> goal_handle
    ) {
      RCLCPP_INFO(this->get_logger(), "Received request to cancel compute grasp validity");
      return rclcpp_action::CancelResponse::REJECT;
    }

    rclcpp_action::GoalResponse handle_goal_open_(
      const rclcpp_action::GoalUUID&, std::shared_ptr<const CustomActionOpen::Goal> goal
    ) {
      RCLCPP_INFO(this->get_logger(), "Received Gripper Open Request on Object %d", (int) goal->target_id);
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel_open_(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<CustomActionOpen>> goal_handle
    ) {
      RCLCPP_INFO(this->get_logger(), "Received request to cancel gripper open");
      return rclcpp_action::CancelResponse::REJECT;
    }

    rclcpp_action::GoalResponse handle_goal_move_(
      const rclcpp_action::GoalUUID&, std::shared_ptr<const CustomActionMove::Goal> goal
    ) {
      RCLCPP_INFO(this->get_logger(), "Received move request");
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel_move_(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<CustomActionMove>> goal_handle
    ) {
      RCLCPP_INFO(this->get_logger(), "Received request to cancel move");
      return rclcpp_action::CancelResponse::REJECT;
    }

    rclcpp_action::GoalResponse handle_goal_pickup_(
      const rclcpp_action::GoalUUID&, std::shared_ptr<const CustomActionPickup::Goal> goal
    ) {
      RCLCPP_INFO(this->get_logger(), "Received pickup request Position");
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel_pickup_(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<CustomActionPickup>> goal_handle
    ) {
      RCLCPP_INFO(this->get_logger(), "Received request to cancel pickup");
      return rclcpp_action::CancelResponse::REJECT;
    }

    rclcpp_action::GoalResponse handle_goal_close_(
      const rclcpp_action::GoalUUID&, std::shared_ptr<const CustomActionClose::Goal> goal
    ) {
      RCLCPP_INFO(this->get_logger(), "Received Gripper Close Request on Object %d", (int)goal->target_id);
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel_close_(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<CustomActionClose>> goal_handle
    ) {
      RCLCPP_INFO(this->get_logger(), "Received request to cancel gripper close");
      return rclcpp_action::CancelResponse::REJECT;
    }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<hts_node>();
  node->init();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  // rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}