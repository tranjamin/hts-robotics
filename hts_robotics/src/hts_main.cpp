#include <chrono>
#include <memory>
#include <string>

// includes
#include "rclcpp/rclcpp.hpp"
#include <rclcpp_action/rclcpp_action.hpp>
#include "std_msgs/msg/string.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include "hts_msgs/action/move_target.hpp"
#include "hts_msgs/action/pick_up_target.hpp"
#include "hts_msgs/action/gripper_open.hpp"
#include "hts_msgs/action/gripper_close.hpp"
#include "hts_msgs/action/grasp_object.hpp"
#include "hts_msgs/action/compute_grasp_validity.hpp"
#include "hts_msgs/srv/request_grasp.hpp"
#include "hts_msgs/srv/enable_orientation_constraints.hpp"
#include "hts_msgs/srv/get_object_position.hpp"

// for using macros like s, ms, us
using namespace std::chrono_literals;

class hts_node : public rclcpp::Node {
public:

  // type definitions
  using CustomActionPickup = hts_msgs::action::PickUpTarget;
  using CustomActionMove = hts_msgs::action::MoveTarget;
  using StampedPoint = geometry_msgs::msg::PointStamped;
  using StampedPose = geometry_msgs::msg::PoseStamped;
  using JointState = sensor_msgs::msg::JointState;
  using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
  using PlanningSceneInterface = moveit::planning_interface::PlanningSceneInterface;
  using PlanningSceneMonitor = planning_scene_monitor::PlanningSceneMonitor;
  using CustomActionOpen = hts_msgs::action::GripperOpen;
  using CustomActionClose = hts_msgs::action::GripperClose;
  using CustomActionComputeGraspValidity = hts_msgs::action::ComputeGraspValidity;

  // constructor
  hts_node():Node("hts_node") {
    RCLCPP_INFO(this->get_logger(), "Constructing HTS Robotics Node...");

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


    // set tolerances for gripper
    gripper_interface_->setGoalPositionTolerance(0.001);
    gripper_interface_->setGoalJointTolerance(0.001);
    gripper_interface_->setGoalOrientationTolerance(0.1);    

    // set tolerances for arm
    move_group_interface_->setGoalPositionTolerance(0.001);
    move_group_interface_->setGoalOrientationTolerance(0.001);
    move_group_interface_->setGoalJointTolerance(0.01);
    move_group_interface_->setPlanningTime(30.0);
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

    // Register the target objects as collision objects
    this->declare_parameter("objects", std::vector<std::string>{});
    std::vector<std::string> objects = this->get_parameter("objects").as_string_array();

    target_object_ids_ = std::vector<std::string>(objects.size(), "");
    int i = 0;

    RCLCPP_DEBUG(get_logger(), "Looking for objects: %ld", objects.size());
    for (auto &obj_name : objects) {
      moveit_msgs::msg::CollisionObject co_target;
      shape_msgs::msg::SolidPrimitive primitive_target;
      geometry_msgs::msg::Pose pose_target;

      this->declare_parameter(obj_name + ".object_id", 0);
      this->declare_parameter(obj_name + ".primitive_type", "");

      int obj_id = this->get_parameter(obj_name + ".object_id").as_int();
      RCLCPP_DEBUG(get_logger(), "Found an Object with ID %d", obj_id);

      std::string obj_type = this->get_parameter(obj_name + ".primitive_type").as_string();
      if (obj_type == "BOX") {
        this->declare_parameter(obj_name + ".primitive_dims.x", 1.0);
        this->declare_parameter(obj_name + ".primitive_dims.y", 1.0);
        this->declare_parameter(obj_name + ".primitive_dims.z", 1.0);

        primitive_target.type = primitive_target.BOX;
        primitive_target.dimensions = {
          this->get_parameter(obj_name + ".primitive_dims.x").as_double(),
          this->get_parameter(obj_name + ".primitive_dims.y").as_double(),
          this->get_parameter(obj_name + ".primitive_dims.z").as_double()
        };
      } else if (obj_type == "SPHERE") {
        this->declare_parameter(obj_name + ".primitive_dims.radius", 1.0);

        primitive_target.type = primitive_target.SPHERE;
        primitive_target.dimensions = {
          this->get_parameter(obj_name + ".primitive_dims.radius").as_double()
        };

      } else if (obj_type == "CONE") {        
        this->declare_parameter(obj_name + ".primitive_dims.height", 1.0);
        this->declare_parameter(obj_name + ".primitive_dims.radius", 1.0);

        primitive_target.type = primitive_target.CONE;
        primitive_target.dimensions = {
          this->get_parameter(obj_name + ".primitive_dims.height").as_double(),
          this->get_parameter(obj_name + ".primitive_dims.radius").as_double()
        };

      } else if (obj_type == "CYLINDER") {
        this->declare_parameter(obj_name + ".primitive_dims.height", 1.0);
        this->declare_parameter(obj_name + ".primitive_dims.radius", 1.0);

        primitive_target.type = primitive_target.CYLINDER;
        primitive_target.dimensions = {
          this->get_parameter(obj_name + ".primitive_dims.height").as_double(),
          this->get_parameter(obj_name + ".primitive_dims.radius").as_double()
        };
      } else {
        RCLCPP_INFO(get_logger(), "Invalid Object");
      }

      this->declare_parameter(obj_name + ".x", 0.0);
      this->declare_parameter(obj_name + ".y", 0.0);
      this->declare_parameter(obj_name + ".z", 0.0);

      this->declare_parameter(obj_name + ".R", 0.0);
      this->declare_parameter(obj_name + ".P", 0.0);
      this->declare_parameter(obj_name + ".Y", 0.0);

      pose_target.position.x = this->get_parameter(obj_name + ".x").as_double();
      pose_target.position.y = this->get_parameter(obj_name + ".y").as_double();
      pose_target.position.z = this->get_parameter(obj_name + ".z").as_double();

      double roll = this->get_parameter(obj_name + ".R").as_double();
      double pitch = this->get_parameter(obj_name + ".P").as_double();
      double yaw = this->get_parameter(obj_name + ".Y").as_double();

      tf2::Quaternion q;
      q.setRPY(roll, pitch, yaw);

      pose_target.orientation.x = q.x();
      pose_target.orientation.y = q.y();
      pose_target.orientation.z = q.z();
      pose_target.orientation.w = q.w();
      
      co_target.id = "target_" + std::to_string(obj_id);
      co_target.header.frame_id = move_group_interface_->getPlanningFrame();
      co_target.primitives.push_back(primitive_target);
      co_target.primitive_poses.push_back(pose_target);
      co_target.operation = co_target.ADD;
      planning_scene_interface_->applyCollisionObject(co_target);

      target_object_ids_[i] = co_target.id;
    }

    planning_scene_monitor_->requestPlanningSceneState();

    move_group_interface_->setPlanningPipelineId("stomp");
    move_group_interface_->setPlannerId("stomp");

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

  private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_update_;
    std::vector<std::string> target_object_ids_;

    // move groups and planning scenes
    std::shared_ptr<MoveGroupInterface> move_group_interface_;
    std::shared_ptr<PlanningSceneInterface> planning_scene_interface_;
    std::shared_ptr<MoveGroupInterface> gripper_interface_;
    std::shared_ptr<PlanningSceneMonitor> planning_scene_monitor_;

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

    void handle_accepted_compute_grasp_validity_(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<CustomActionComputeGraspValidity>> goal_handle
    ) {
      std::thread([this, goal_handle] {
        
      RCLCPP_INFO(this->get_logger(), "Computing grasp validity");
      move_group_interface_->setPlanningPipelineId("stomp");

      geometry_msgs::msg::Pose goal_pose = goal_handle->get_goal()->grasp_pose;
      auto result = std::make_shared<CustomActionComputeGraspValidity::Result>();

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      move_group_interface_->setStartStateToCurrentState();
      move_group_interface_->setPoseTarget(goal_pose);

      bool success = (move_group_interface_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
      // bool success = true;
      if (!success) {
        RCLCPP_INFO(this->get_logger(), "Planning failed");
        result->success = true;
        result->is_valid = false;
        result->score = 0.0;
        result->message = "Plan is impossible";
        goal_handle->succeed(result);
      } else {
        RCLCPP_INFO(this->get_logger(), "Planning succeeded");
        result->success = true;
        result->is_valid = true;
        result->score = 10.0;
        result->message = "Plan is possible";
        goal_handle->succeed(result);
      }

      RCLCPP_INFO(this->get_logger(), "After goal handle succeed");

      }).detach();

    }
    

    void handle_accepted_close_(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<CustomActionClose>> goal_handle
    ) {
        RCLCPP_INFO(get_logger(), "\n\n\n\n--------------- CLOSE CALLBACK ---------------\n\n\n\n");
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
        gripper_interface_->attachObject(object_name);

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
    }

    void handle_accepted_open_(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<CustomActionOpen>> goal_handle
    ) {
        RCLCPP_INFO(get_logger(), "\n\n\n\n--------------- OPEN CALLBACK ---------------\n\n\n\n");
        
        auto object_name = "target_" + std::to_string(goal_handle->get_goal()->target_id);

        gripper_interface_->setNamedTarget("open");

        // detach object
        gripper_interface_->detachObject(object_name);

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
    }

    void handle_accepted_pickup_(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<CustomActionPickup>> goal_handle
    ) {
        RCLCPP_INFO(get_logger(), "\n\n\n\n--------------- PICKUP CALLBACK ---------------\n\n\n\n");

        // set the goal
        auto goal = goal_handle->get_goal();
        geometry_msgs::msg::Pose target;
        target.position.x = goal->x;
        target.position.y = goal->y;
        target.position.z = goal->z;

        tf2::Quaternion q;
        q.setRPY(goal->ox, goal->oy, goal->oz);

        target.orientation.x = goal->ox;
        target.orientation.y = goal->oy;
        target.orientation.z = goal->oz;
        target.orientation.w = goal->ow;

        RCLCPP_INFO(this->get_logger(), "Target Position is (%.2f, %.2f, %.2f)", goal->x, goal->y, goal->z);
        RCLCPP_INFO(this->get_logger(), "Target Angle is (%.2f, %.2f, %.2f)", goal->ox, goal->oy, goal->oz);
        RCLCPP_INFO(this->get_logger(), "Target Quaternion is (%.2f, %.2f, %.2f, %.2f)", q.x(), q.y(), q.z(), q.w());

        
        auto current_state = move_group_interface_->getCurrentState();


        move_group_interface_->setStartStateToCurrentState();
        move_group_interface_->setPlanningPipelineId("stomp");

        move_group_interface_->clearPathConstraints();
        RCLCPP_INFO(this->get_logger(), "Cleared Path Constraints");
        move_group_interface_->setPoseTarget(target);
        bool success = (move_group_interface_->move() == moveit::core::MoveItErrorCode::SUCCESS);

        auto current_position = move_group_interface_->getCurrentPose().pose;
        RCLCPP_INFO(this->get_logger(), "End Position is (%.2f, %.2f, %.2f)", 
          current_position.position.x, current_position.position.y, current_position.position.z);
        RCLCPP_INFO(this->get_logger(), "End Quaternion is (%.2f, %.2f, %.2f, %.2f)",
          current_position.orientation.x, current_position.orientation.y, current_position.orientation.z, current_position.orientation.w);
        
        auto result = std::make_shared<CustomActionPickup::Result>();
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

        RCLCPP_INFO(get_logger(), "\n\n\n\n--------------- PICKUP CALLBACK END ---------------\n\n\n\n");
    
    }

    void handle_accepted_move_(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<CustomActionMove>> goal_handle
    ) {
        RCLCPP_INFO(get_logger(), "\n\n\n\n--------------- MOVE CALLBACK ---------------\n\n\n\n");

        auto goal = goal_handle->get_goal();

        // Apply orientation constraints
        moveit_msgs::msg::OrientationConstraint orientation_constraint;
        orientation_constraint.header.frame_id = move_group_interface_->getPoseReferenceFrame();
        orientation_constraint.link_name = move_group_interface_->getEndEffectorLink();

        auto current_pose = move_group_interface_->getCurrentPose();
        orientation_constraint.orientation = current_pose.pose.orientation;
        RCLCPP_INFO(get_logger(), "Current Pose Orientation: (%f, %f, %f, %f)",
          current_pose.pose.orientation.x, current_pose.pose.orientation.y,
          current_pose.pose.orientation.z, current_pose.pose.orientation.w
        );

        geometry_msgs::msg::Pose target_pose;
        target_pose.orientation.x = current_pose.pose.orientation.x;
        target_pose.orientation.y = current_pose.pose.orientation.y;
        target_pose.orientation.z = current_pose.pose.orientation.z;
        target_pose.orientation.w = current_pose.pose.orientation.w;
        target_pose.position.x = goal->x;
        target_pose.position.y = goal->y;
        target_pose.position.z = goal->z;

        orientation_constraint.absolute_x_axis_tolerance = 0.2;
        orientation_constraint.absolute_y_axis_tolerance = 0.2;
        orientation_constraint.absolute_z_axis_tolerance = 10;
        orientation_constraint.weight = 1.0;
        orientation_constraint.parameterization = orientation_constraint.ROTATION_VECTOR;

        moveit_msgs::msg::Constraints all_constraints;
        all_constraints.orientation_constraints.emplace_back(orientation_constraint);

        move_group_interface_->clearPathConstraints();
        move_group_interface_->setPathConstraints(all_constraints);
        RCLCPP_INFO(get_logger(), "Applied orientation constraints to planning scene.");

        RCLCPP_INFO(this->get_logger(), "Target Position is (%.2f, %.2f, %.2f)", goal->x, goal->y, goal->z);

        move_group_interface_->getCurrentState();
        move_group_interface_->setStartStateToCurrentState();
        move_group_interface_->setPlanningPipelineId("ompl");
        
        // move_group_interface_->setPositionTarget(goal->x, goal->y, goal->z);
        move_group_interface_->setPoseTarget(target_pose);
        bool success = (move_group_interface_->move() == moveit::core::MoveItErrorCode::SUCCESS);

        auto current_position = move_group_interface_->getCurrentPose().pose;
        double roll2, pitch2, yaw2;

        RCLCPP_INFO(this->get_logger(), "End Position is (%.2f, %.2f, %.2f)", 
          current_position.position.x, current_position.position.y, current_position.position.z);
        RCLCPP_INFO(this->get_logger(), "End Quaternion is (%.2f, %.2f, %.2f, %.2f)",
          current_position.orientation.x, current_position.orientation.y, current_position.orientation.z, current_position.orientation.w);

        move_group_interface_->clearPathConstraints();

        auto result = std::make_shared<CustomActionMove::Result>();
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

        RCLCPP_INFO(get_logger(), "\n\n\n\n--------------- MOVE CALLBACK END ---------------\n\n\n\n");
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
      RCLCPP_INFO(this->get_logger(), "Received move request (%.2f %.2f %.2f)", goal->x, goal->y, goal->z);
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
      RCLCPP_INFO(this->get_logger(), "Received pickup request Position (%.2f %.2f %.2f) and Orientation (%.2f %.2f %.2f %.2f)", goal->x, goal->y, goal->z, goal->ox, goal->oy, goal->oz, goal->ow);
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