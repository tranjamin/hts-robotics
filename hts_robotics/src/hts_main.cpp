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

#include "hts_robotics/action/move_target.hpp"
#include "hts_robotics/action/pick_up_target.hpp"
#include "hts_robotics/action/gripper_open.hpp"
#include "hts_robotics/action/gripper_close.hpp"
#include "hts_robotics/action/grasp_object.hpp"
#include "hts_msgs/srv/request_grasp.hpp"

// for using macros like s, ms, us
using namespace std::chrono_literals;


/**
 * a class for high-level control for high throughput synthesis robotics
 * 
 * Node:
 *  - hts_node
 * Subscribes to:
 *  - /clicked_point: moves the robot to predefined point
 *  - /goal_pose: logs the goal pose
 *  - /joint_states: logs the joint states (debug)
 * Publishes:
 *  - /goal_pose: publishes a fixed goal pose
 * Action Servers:
 *  - /hts_moveit_action: publishes hts commands to pass to moveit
 * Action Clients:
 *  - /hts_moveit_action: reads in hts commands and forwards it to moveit
 */
class hts_node : public rclcpp::Node {
public:

  // type definitions
  using CustomActionPickup = hts_robotics::action::PickUpTarget;
  using CustomActionMove = hts_robotics::action::MoveTarget;
  using StampedPoint = geometry_msgs::msg::PointStamped;
  using StampedPose = geometry_msgs::msg::PoseStamped;
  using JointState = sensor_msgs::msg::JointState;
  using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
  using PlanningSceneInterface = moveit::planning_interface::PlanningSceneInterface;
  using PlanningSceneMonitor = planning_scene_monitor::PlanningSceneMonitor;
  using CustomActionOpen = hts_robotics::action::GripperOpen;
  using CustomActionClose = hts_robotics::action::GripperClose;
  using CustomActionGraspObject = hts_robotics::action::GraspObject;

  // constructor
  hts_node():Node("hts_node") {
    RCLCPP_INFO(this->get_logger(), "Constructing HTS Robotics Node...");

    // create a subscriber to read in the joint states and log info
    RCLCPP_INFO(this->get_logger(), "Creating joint states subscriber...");
    joint_states_sub_ = this->create_subscription<JointState>(
      "/joint_states", 10,
      std::bind(&hts_node::joint_states_subscriber_callback_, this, std::placeholders::_1)
    );
    RCLCPP_INFO(this->get_logger(), "Created joint states subscriber.");

    // create a subscriber for the gazebo scene
    RCLCPP_INFO(this->get_logger(), "Creating Gazebo scene subscriber...");
    gazebo_scene_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
      "/world/empty/dynamic_pose/info", 1,
      std::bind(&hts_node::gazebo_scene_subscriber_callback_, this, std::placeholders::_1)
    );
    RCLCPP_INFO(this->get_logger(), "Created Gazebo scene subscriber.");

    // create pickup server
    RCLCPP_INFO(this->get_logger(), "Creating Pickup Server...");
    pickup_server_ = rclcpp_action::create_server<CustomActionPickup>(
      this, "hts_pickup_action",
      std::bind(&hts_node::handle_goal_pickup_, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&hts_node::handle_cancel_pickup_, this, std::placeholders::_1),
      std::bind(&hts_node::handle_accepted_pickup_, this, std::placeholders::_1)
    );
    RCLCPP_INFO(this->get_logger(), "Created Pickup Server.");

    // create move server
    RCLCPP_INFO(this->get_logger(), "Creating Move Server...");
    move_server_ = rclcpp_action::create_server<CustomActionMove>(
      this, "hts_move_action",
      std::bind(&hts_node::handle_goal_move_, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&hts_node::handle_cancel_move_, this, std::placeholders::_1),
      std::bind(&hts_node::handle_accepted_move_, this, std::placeholders::_1)
    );
    RCLCPP_INFO(this->get_logger(), "Created Move Server.");

    // create gripper servers
    RCLCPP_INFO(this->get_logger(), "Creating Gripper Servers...");
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
    RCLCPP_INFO(this->get_logger(), "Created Gripper Servers.");

    // create grasping servers
    RCLCPP_INFO(this->get_logger(), "Creating Grasping Servers...");
    grasp_object_server_ = rclcpp_action::create_server<CustomActionGraspObject>(
      this, "grasp_object",
      std::bind(&hts_node::handle_goal_grasp_object_, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&hts_node::handle_cancel_grasp_object_, this, std::placeholders::_1),
      std::bind(&hts_node::handle_accepted_grasp_object_, this, std::placeholders::_1)
    );
    grasp_request_client_ = this->create_client<hts_msgs::srv::RequestGrasp>("/request_grasp");
    grasp_request_client_->wait_for_service();
    
    pickup_client_ = rclcpp_action::create_client<CustomActionPickup>(this, "hts_pickup_action");
    move_client_ = rclcpp_action::create_client<CustomActionMove>(this, "hts_move_action");
    open_client_ = rclcpp_action::create_client<CustomActionOpen>(this, "gripper_open");
    close_client_ = rclcpp_action::create_client<CustomActionClose>(this, "gripper_close");

    RCLCPP_INFO(this->get_logger(), "Created Grasper Clients and Servers.");

    RCLCPP_INFO(this->get_logger(), "Finished constructing HTS Node.");
  }

  // initialise
  void init() {
    last_update_ = this->now();

    RCLCPP_INFO(this->get_logger(), "Starting HTS node initialisation...");

    move_group_interface_ = std::make_shared<MoveGroupInterface>(shared_from_this(), "fr3_arm");
    gripper_interface_ = std::make_shared<MoveGroupInterface>(shared_from_this(), "fr3_hand");
    planning_scene_interface_ = std::make_shared<PlanningSceneInterface>();
    planning_scene_monitor_ = std::make_shared<PlanningSceneMonitor>(shared_from_this(), "robot_description");
    RCLCPP_INFO(this->get_logger(), "Initialised move groups and planning scenes.");

    // start planning scene monitor
    planning_scene_monitor_->startSceneMonitor();
    planning_scene_monitor_->startWorldGeometryMonitor();
    planning_scene_monitor_->startStateMonitor();

    // set tolerances
    gripper_interface_->setGoalPositionTolerance(0.001);
    gripper_interface_->setGoalJointTolerance(0.001);

    // for dynamically updating the planning scene (enabling and disabling collisions)
    planning_scene_diff_publisher_ = this->create_publisher<moveit_msgs::msg::PlanningScene>("/planning_scene", 10);

    // Register the ground as a collision object
    moveit_msgs::msg::CollisionObject co_ground;
    shape_msgs::msg::SolidPrimitive primitive_ground;
    geometry_msgs::msg::Pose pose_ground;

    primitive_ground.type = primitive_ground.BOX;
    primitive_ground.dimensions = {10, 10, 0.25};

    pose_ground.orientation.w = 1.0;
    pose_ground.position.x = 0;
    pose_ground.position.y = 0;
    pose_ground.position.z = -0.25;

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

    RCLCPP_INFO(get_logger(), "Looking for objects: %ld", objects.size());
    for (auto &obj_name : objects) {
      RCLCPP_INFO(get_logger(), "Found object");
      moveit_msgs::msg::CollisionObject co_target;
      shape_msgs::msg::SolidPrimitive primitive_target;
      geometry_msgs::msg::Pose pose_target;

      this->declare_parameter(obj_name + ".object_id", 0);
      this->declare_parameter(obj_name + ".primitive_type", "");

      int obj_id = this->get_parameter(obj_name + ".object_id").as_int();
      RCLCPP_INFO(get_logger(), "Found an Object with ID %d", obj_id);

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
  rclcpp::Subscription<JointState>::SharedPtr joint_states_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr gazebo_scene_sub_;
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_diff_publisher_;
  
  // action servers and clients
  rclcpp_action::Server<CustomActionPickup>::SharedPtr pickup_server_;
  rclcpp_action::Server<CustomActionMove>::SharedPtr move_server_;
  rclcpp_action::Server<CustomActionOpen>::SharedPtr gripper_open_server_;
  rclcpp_action::Server<CustomActionClose>::SharedPtr gripper_close_server_;

  // for the top-level actions
  rclcpp_action::Server<CustomActionGraspObject>::SharedPtr grasp_object_server_;
  rclcpp::Client<hts_msgs::srv::RequestGrasp>::SharedPtr grasp_request_client_;
  rclcpp_action::Client<CustomActionPickup>::SharedPtr pickup_client_;
  rclcpp_action::Client<CustomActionMove>::SharedPtr move_client_;
  rclcpp_action::Client<CustomActionOpen>::SharedPtr open_client_;
  rclcpp_action::Client<CustomActionClose>::SharedPtr close_client_;

  // // Server Callbacks
  rclcpp_action::GoalResponse handle_goal_grasp_object_(
    const rclcpp_action::GoalUUID&, std::shared_ptr<const CustomActionGraspObject::Goal> goal
  ) {
    RCLCPP_INFO(this->get_logger(), "Received Grasp Object Request");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel_grasp_object_(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<CustomActionGraspObject>> goal_handle
  ) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel grasp object");
    return rclcpp_action::CancelResponse::REJECT;
  }
  void handle_accepted_grasp_object_(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<CustomActionGraspObject>> goal_handle
  ) {
    std::thread([this, goal_handle] () {
      RCLCPP_INFO(this->get_logger(), "TESTING");
      
      // the result and feedback objects
      auto result = std::make_shared<CustomActionGraspObject::Result>();
      auto progress = std::make_shared<CustomActionGraspObject::Feedback>();

      // get the grasp for the target object
      auto grasp_request = std::make_shared<hts_msgs::srv::RequestGrasp::Request>();
      auto object_id = goal_handle->get_goal()->object_id;
      auto object_name = "target_" + std::to_string(object_id);

      auto map = planning_scene_interface_->getObjectPoses({object_name});
      if (map.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Could not find object in planning scene");
        return;
      }
      geometry_msgs::msg::Pose target_moveit = map.at(object_name);

      grasp_request->id = object_id;
      grasp_request->x = target_moveit.position.x;
      grasp_request->y = target_moveit.position.y;
      grasp_request->z = target_moveit.position.z;

      auto grasp_future = grasp_request_client_->async_send_request(grasp_request);
      auto grasp_response = grasp_future.get();

      // if anygrasp failed
      if (!grasp_response->success) {
        RCLCPP_ERROR(this->get_logger(), "Anygrasp failed to identify pose");
        result->success = false;
        result->message = "Anygrasp failed to identify pose";
        goal_handle->abort(result);
        return;
      }

      geometry_msgs::msg::Pose grasp_pose = grasp_response->grasp_pose;

      // update progress
      RCLCPP_INFO(this->get_logger(), "AnyGrasp Found a Grasp of (%.2f, %.2f, %.2f) (%.2f, %.2f, %.2f, %.2f)", 
        grasp_pose.position.x, grasp_pose.position.y, grasp_pose.position.z,
        grasp_pose.orientation.x, grasp_pose.orientation.y, grasp_pose.orientation.z, grasp_pose.orientation.w
      );
      char buf[150];
      std::snprintf(buf, sizeof(buf),
        "AnyGrasp Found a Grasp of (%.2f, %.2f, %.2f) (%.2f, %.2f, %.2f, %.2f)", 
        grasp_pose.position.x, grasp_pose.position.y, grasp_pose.position.z,
        grasp_pose.orientation.x, grasp_pose.orientation.y, grasp_pose.orientation.z, grasp_pose.orientation.w
      );
      progress->progress = std::string(buf);
      goal_handle->publish_feedback(progress);


      auto pickup_goal = CustomActionPickup::Goal();
      pickup_goal.x = grasp_pose.position.x;
      pickup_goal.y = grasp_pose.position.y;
      pickup_goal.z = grasp_pose.position.z;
      pickup_goal.ox = grasp_pose.orientation.x;
      pickup_goal.oy = grasp_pose.orientation.y;
      pickup_goal.oz = grasp_pose.orientation.z;
      pickup_goal.ow = grasp_pose.orientation.w;

      auto open_send_goal_options = rclcpp_action::Client<CustomActionOpen>::SendGoalOptions();
      open_send_goal_options.result_callback =
        [this, goal_handle, result, progress](const rclcpp_action::ClientGoalHandle<CustomActionOpen>::WrappedResult &r) {
          if (r.code != rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_ERROR(this->get_logger(), "Gripper failed to open with code: %d", (int)r.code);
            result->success = false;
            result->message = "Gripper failed to open";
            goal_handle->abort(result);
          } else {
            RCLCPP_INFO(this->get_logger(), "Gripper opened on target");
            progress->progress = "Gripper opened on target";
            goal_handle->publish_feedback(progress);
          }
        };    

      auto move_send_goal_options = rclcpp_action::Client<CustomActionMove>::SendGoalOptions();
      move_send_goal_options.result_callback =
        [this, open_send_goal_options, goal_handle, result, progress](const rclcpp_action::ClientGoalHandle<CustomActionMove>::WrappedResult &r) {
          if (r.code != rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_ERROR(this->get_logger(), "MoveIt failed to move with code: %d", (int)r.code);
            result->success = false;
            result->message = "MoveIt failed to move";
            goal_handle->abort(result);
          } else {
            RCLCPP_INFO(this->get_logger(), "MoveIt moved target");
            progress->progress = "MoveIt moved target";
            goal_handle->publish_feedback(progress);

            auto open_goal = CustomActionOpen::Goal();
            open_client_->async_send_goal(open_goal, open_send_goal_options);
          }
        };

      auto close_send_goal_options = rclcpp_action::Client<CustomActionClose>::SendGoalOptions();
      close_send_goal_options.result_callback =
        [this, move_send_goal_options, goal_handle, result, progress](const rclcpp_action::ClientGoalHandle<CustomActionClose>::WrappedResult &r) {
          if (r.code != rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_ERROR(this->get_logger(), "Gripper failed to close with code: %d", (int)r.code);
            result->success = false;
            result->message = "Gripper failed to close";
            goal_handle->abort(result);
          } else {
            RCLCPP_INFO(this->get_logger(), "Gripper closed on target");
            progress->progress = "Gripper closed on target";
            goal_handle->publish_feedback(progress);

            auto move_goal = CustomActionMove::Goal();
            move_goal.x = goal_handle->get_goal()->x;
            move_goal.y = goal_handle->get_goal()->y;
            move_goal.z = goal_handle->get_goal()->z;

            move_client_->async_send_goal(move_goal, move_send_goal_options);
          }
        };

      auto pickup_send_goal_options = rclcpp_action::Client<CustomActionPickup>::SendGoalOptions();
      pickup_send_goal_options.result_callback =
        [this, close_send_goal_options, goal_handle, result, progress](const rclcpp_action::ClientGoalHandle<CustomActionPickup>::WrappedResult &r) {
          if (r.code != rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_ERROR(this->get_logger(), "Movet failed to move to target with code: %d", (int)r.code);
            result->success = false;
            result->message = "MoveIt failed to move to target";
            goal_handle->abort(result);
          } else {
            RCLCPP_INFO(this->get_logger(), "MoveIt moved to target");            
            progress->progress = "MoveIt moved to target";
            goal_handle->publish_feedback(progress);

            auto close_goal = CustomActionClose::Goal();
            close_client_->async_send_goal(close_goal, close_send_goal_options);
          }
        };

      // sends action
      pickup_client_->async_send_goal(pickup_goal, pickup_send_goal_options);
    }).detach();
  }

  // Server Callbacks for Gripper Close Action
  rclcpp_action::GoalResponse handle_goal_close_(
    const rclcpp_action::GoalUUID&, std::shared_ptr<const CustomActionClose::Goal> goal
  ) {
    RCLCPP_INFO(this->get_logger(), "Received Gripper Close Request");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel_close_(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<CustomActionClose>> goal_handle
  ) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel gripper close");
    return rclcpp_action::CancelResponse::REJECT;
  }

  void handle_accepted_close_(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<CustomActionClose>> goal_handle
  ) {
    std::thread([this, goal_handle]() {
      gripper_interface_->setNamedTarget("close");

      // disable collisions
      planning_scene_monitor::LockedPlanningSceneRW scene(planning_scene_monitor_);
      auto &acm = scene->getAllowedCollisionMatrixNonConst();
      acm.setEntry("target", "fr3_hand", true);
      acm.setEntry("target", "fr3_leftfinger", true);
      acm.setEntry("target", "fr3_rightfinger", true);
      moveit_msgs::msg::PlanningScene ps_msg;
      ps_msg.is_diff = true;
      scene->getPlanningSceneMsg(ps_msg);
      planning_scene_interface_->applyPlanningScene(ps_msg);

      // move
      bool success = (gripper_interface_->move() == moveit::core::MoveItErrorCode::SUCCESS);

      // attach object
      gripper_interface_->attachObject("target");

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
    }).detach();
  }

  // Server Callbacks for Gripper Open Action
  rclcpp_action::GoalResponse handle_goal_open_(
    const rclcpp_action::GoalUUID&, std::shared_ptr<const CustomActionOpen::Goal> goal
  ) {
    RCLCPP_INFO(this->get_logger(), "Received Gripper Open Request");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel_open_(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<CustomActionOpen>> goal_handle
  ) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel gripper open");
    return rclcpp_action::CancelResponse::REJECT;
  }

  void handle_accepted_open_(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<CustomActionOpen>> goal_handle
  ) {
    std::thread([this, goal_handle]() {
      gripper_interface_->setNamedTarget("open");

      // detach object
      gripper_interface_->detachObject("target");

      // move
      bool success = (gripper_interface_->move() == moveit::core::MoveItErrorCode::SUCCESS);

      // enable collisions
      planning_scene_monitor::LockedPlanningSceneRW scene(planning_scene_monitor_);
      auto &acm = scene->getAllowedCollisionMatrixNonConst();
      acm.setEntry("target", "fr3_hand", false);
      acm.setEntry("target", "fr3_leftfinger", false);
      acm.setEntry("target", "fr3_rightfinger", false);
      moveit_msgs::msg::PlanningScene ps_msg;
      ps_msg.is_diff = true;
      scene->getPlanningSceneMsg(ps_msg);
      planning_scene_interface_->applyPlanningScene(ps_msg);

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
    }).detach();
  }

  // Server Callbacks for Pickup Target Action
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

  void handle_accepted_pickup_(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<CustomActionPickup>> goal_handle
  ) {
    std::thread([this, goal_handle]() {
      auto goal = goal_handle->get_goal();

      geometry_msgs::msg::Pose target;
      target.position.x = goal->x;
      target.position.y = goal->y;
      target.position.z = goal->z;

      tf2::Quaternion q;
      q.setRPY(goal->ox, goal->oy, goal->oz);

      target.orientation.x = q.x();
      target.orientation.y = q.y();
      target.orientation.z = q.z();
      target.orientation.w = q.w();

      RCLCPP_INFO(this->get_logger(), "Target Position is (%.2f, %.2f, %.2f)", goal->x, goal->y, goal->z);
      RCLCPP_INFO(this->get_logger(), "Target Angle is (%.2f, %.2f, %.2f)", goal->ox, goal->oy, goal->oz);
      RCLCPP_INFO(this->get_logger(), "Target Quaternion is (%.2f, %.2f, %.2f, %.2f)", q.x(), q.y(), q.z(), q.w());

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

    }).detach();
  
  }

  // Server Callbacks for Move Target Action
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

  void handle_accepted_move_(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<CustomActionMove>> goal_handle
  ) {
    std::thread([this, goal_handle]() {
      auto goal = goal_handle->get_goal();

      geometry_msgs::msg::Pose target;
      target.position.x = goal->x;
      target.position.y = goal->y;
      target.position.z = goal->z;

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
      orientation_constraint.absolute_x_axis_tolerance = 0.3;
      orientation_constraint.absolute_y_axis_tolerance = 0.3;
      orientation_constraint.absolute_z_axis_tolerance = 1000;
      orientation_constraint.weight = 1.0;

      moveit_msgs::msg::Constraints all_constraints;
      all_constraints.orientation_constraints.emplace_back(orientation_constraint);

      move_group_interface_->setPathConstraints(all_constraints);
      RCLCPP_INFO(get_logger(), "Applied orientation constraints to planning scene.");

      target.orientation.x = current_pose.pose.orientation.x;
      target.orientation.y = current_pose.pose.orientation.y;
      target.orientation.z = current_pose.pose.orientation.z;
      target.orientation.w = current_pose.pose.orientation.w;

      RCLCPP_INFO(this->get_logger(), "Target Position is (%.2f, %.2f, %.2f)", goal->x, goal->y, goal->z);
      RCLCPP_INFO(this->get_logger(), "Target Quaternion is (%.2f, %.2f, %.2f, %.2f)", target.orientation.x, target.orientation.y, target.orientation.z, target.orientation.w);

      move_group_interface_->setPoseTarget(target);
      bool success = (move_group_interface_->move() == moveit::core::MoveItErrorCode::SUCCESS);

      auto current_position = move_group_interface_->getCurrentPose().pose;
      double roll2, pitch2, yaw2;

      RCLCPP_INFO(this->get_logger(), "End Position is (%.2f, %.2f, %.2f)", 
        current_position.position.x, current_position.position.y, current_position.position.z);
      RCLCPP_INFO(this->get_logger(), "End Quaternion is (%.2f, %.2f, %.2f, %.2f)",
        current_position.orientation.x, current_position.orientation.y, current_position.orientation.z, current_position.orientation.w);
      
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
    }).detach();
  }

  void joint_states_subscriber_callback_(JointState::UniquePtr msg) {
    std::ostringstream oss;
    oss << "JointState: \n";
    for (size_t i = 0; i < msg->name.size(); ++i) {
      oss << "  " << msg->name[i] << ": ";
      if (i < msg->position.size()) oss << "pos=" << msg->position[i] << " ";
      if (i < msg->velocity.size()) oss << "vel=" << msg->velocity[i] << " ";
      if (i < msg->effort.size())   oss << "eff=" << msg->effort[i] << " ";
      oss << "\n";
    }
    RCLCPP_DEBUG(this->get_logger(), "%s", oss.str().c_str());
  }

  void gazebo_scene_subscriber_callback_(tf2_msgs::msg::TFMessage::UniquePtr msg) {
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
        ) return;


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

        return;
      }
    }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<hts_node>();
  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}