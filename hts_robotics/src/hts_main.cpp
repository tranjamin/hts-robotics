#include <chrono>
#include <memory>
#include <string>

// includes
#include "rclcpp/rclcpp.hpp"
#include <rclcpp_action/rclcpp_action.hpp>
#include "std_msgs/msg/string.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <tf2/LinearMath/Quaternion.h>

#include "hts_robotics/action/move_to_point.hpp"
#include "hts_robotics/action/move_target.hpp"
#include "hts_robotics/action/pick_up_target.hpp"
#include "hts_robotics/action/gripper_open.hpp"
#include "hts_robotics/action/gripper_close.hpp"

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
  using CustomActionPoint = hts_robotics::action::MoveToPoint;
  using CustomActionPickup = hts_robotics::action::PickUpTarget;
  using CustomActionMove = hts_robotics::action::MoveTarget;
  using StampedPoint = geometry_msgs::msg::PointStamped;
  using StampedPose = geometry_msgs::msg::PoseStamped;
  using JointState = sensor_msgs::msg::JointState;
  using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
  using PlanningSceneInterface = moveit::planning_interface::PlanningSceneInterface;
  using CustomActionOpen = hts_robotics::action::GripperOpen;
  using CustomActionClose = hts_robotics::action::GripperClose;

  // constructor
  hts_node():Node("hts_node") {
    RCLCPP_INFO(this->get_logger(), "Constructing HTS Robotics Node...");

    // create a subscriber to take in a clicked point and move the robot
    RCLCPP_INFO(this->get_logger(), "Creating clicked point subscriber...");
    clicked_point_sub_ = this->create_subscription<StampedPoint>(
      "/clicked_point", 10,
      std::bind(&hts_node::clicked_point_subscriber_callback_, this, std::placeholders::_1)
    );
    RCLCPP_INFO(this->get_logger(), "Created clicked point subscriber.");

    // create a subscriber to read in the goal pose and log info
    RCLCPP_INFO(this->get_logger(), "Creating goal pose subscriber...");
    goal_pose_sub_ = this->create_subscription<StampedPose>(
      "/goal_pose", 10,
      std::bind(&hts_node::goal_pose_subscriber_callback_, this, std::placeholders::_1)
    );
    RCLCPP_INFO(this->get_logger(), "Created goal pose subscriber.");

    // create a subscriber to read in the joint states and log info
    RCLCPP_INFO(this->get_logger(), "Creating joint states subscriber...");
    joint_states_sub_ = this->create_subscription<JointState>(
      "/joint_states", 10,
      std::bind(&hts_node::joint_states_subscriber_callback_, this, std::placeholders::_1)
    );
    RCLCPP_INFO(this->get_logger(), "Created joint states subscriber.");

    // create a publisher for the goal pose
    RCLCPP_INFO(this->get_logger(), "Creating goal pose publisher...");
    goal_pose_pub_ = this->create_publisher<StampedPose>("/goal_pose", 10);
    timer_ = this->create_wall_timer(
      500ms,
      std::bind(&hts_node::goal_pose_publisher_callback_, this)
    );
    RCLCPP_INFO(this->get_logger(), "Created goal pose publisher.");

    // create moveit server
    RCLCPP_INFO(this->get_logger(), "Creating MoveIt2 Server...");
    moveit_server_ = rclcpp_action::create_server<CustomActionPoint>(
      this, "hts_moveit_action",
      std::bind(&hts_node::handle_goal_point_, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&hts_node::handle_cancel_point_, this, std::placeholders::_1),
      std::bind(&hts_node::handle_accepted_point_, this, std::placeholders::_1)
    );
    RCLCPP_INFO(this->get_logger(), "Created MoveIt2 Server.");

    RCLCPP_INFO(this->get_logger(), "Creating MoveIt2 Client...");
    moveit_client_ = rclcpp_action::create_client<CustomActionPoint>(this, "hts_moveit_action");    
    if (!moveit_client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to connect to action server");
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Created MoveIt2 Client.");

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

    // create move server
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

    RCLCPP_INFO(this->get_logger(), "Finished constructing HTS Node.");
  }

  void init() {
    RCLCPP_INFO(this->get_logger(), "Starting HTS node initialisation...");
    RCLCPP_INFO(this->get_logger(), "Initialising MoveGroup and Planning SceneInterface...");
    move_group_interface_ = std::make_shared<MoveGroupInterface>(shared_from_this(), "fr3_arm");
    planning_scene_interface_ = std::make_shared<PlanningSceneInterface>();
    RCLCPP_INFO(get_logger(), "MoveGroup and PlanningSceneInterface initialized. Planning frame: %s",
                move_group_interface_->getPlanningFrame().c_str());

    RCLCPP_INFO(this->get_logger(), "Initialising Gripper MoveGroup...");
    gripper_interface_ = std::make_shared<MoveGroupInterface>(shared_from_this(), "fr3_hand");
    gripper_interface_->setGoalPositionTolerance(0.001);
    gripper_interface_->setGoalJointTolerance(0.001);


    // Build collision object now that move_group_interface_ exists
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.id = "box1";
    collision_object.header.frame_id = move_group_interface_->getPlanningFrame();

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions = {0.5, 0.1, 0.5}; // meters

    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.2;
    box_pose.position.y = 0.2;
    box_pose.position.z = 0.25;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    // Apply the object
    // planning_scene_interface_->applyCollisionObject(collision_object);
    RCLCPP_INFO(get_logger(), "Applied collision object 'box1' to planning scene.");

    // Apply orientation constraints
    moveit_msgs::msg::OrientationConstraint orientation_constraint;
    // orientation_constraint.header.frame_id = move_group_interface_->getPoseReferenceFrame();
    // orientation_constraint.link_name = move_group_interface_->getEndEffectorLink();
    orientation_constraint.header.frame_id = "world"; // or base
    orientation_constraint.link_name = "fr3_hand"; // or fr3_link8

    RCLCPP_INFO(get_logger(), "EEF link: %s", move_group_interface_->getEndEffectorLink().c_str());
    RCLCPP_INFO(get_logger(), "REF frame: %s", move_group_interface_->getPoseReferenceFrame().c_str());

    auto current_pose = move_group_interface_->getCurrentPose();
    orientation_constraint.orientation = current_pose.pose.orientation;
    RCLCPP_INFO(get_logger(), "Current Pose Orientation: (%f, %f, %f, %f)",
      current_pose.pose.orientation.x, current_pose.pose.orientation.y,
      current_pose.pose.orientation.z, current_pose.pose.orientation.w
    );
    orientation_constraint.absolute_x_axis_tolerance = 0.02;
    orientation_constraint.absolute_y_axis_tolerance = 0.02;
    orientation_constraint.absolute_z_axis_tolerance = 0.02;
    orientation_constraint.weight = 1.0;

    moveit_msgs::msg::Constraints all_constraints;
    all_constraints.orientation_constraints.emplace_back(orientation_constraint);

    move_group_interface_->setPathConstraints(all_constraints);
    RCLCPP_INFO(get_logger(), "Applied orientation constraints to planning scene.");
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<MoveGroupInterface> move_group_interface_;
  std::shared_ptr<PlanningSceneInterface> planning_scene_interface_;
  std::shared_ptr<MoveGroupInterface> gripper_interface_;

  rclcpp::Subscription<StampedPoint>::SharedPtr clicked_point_sub_;
  rclcpp::Subscription<StampedPose>::SharedPtr goal_pose_sub_;
  rclcpp::Subscription<JointState>::SharedPtr joint_states_sub_;
  rclcpp::Publisher<StampedPose>::SharedPtr goal_pose_pub_;

  rclcpp_action::Client<CustomActionPoint>::SharedPtr moveit_client_;
  rclcpp_action::Server<CustomActionPoint>::SharedPtr moveit_server_;

  rclcpp_action::Server<CustomActionPickup>::SharedPtr pickup_server_;
  rclcpp_action::Server<CustomActionMove>::SharedPtr move_server_;

  rclcpp_action::Server<CustomActionOpen>::SharedPtr gripper_open_server_;
  rclcpp_action::Server<CustomActionClose>::SharedPtr gripper_close_server_;

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

      auto joints = gripper_interface_->getCurrentJointValues();
      for (size_t i = 0; i < joints.size(); ++i)
        RCLCPP_INFO(this->get_logger(), "Gripper %zu: %f", i, joints[i]);

      auto joint_values = gripper_interface_->getNamedTargetValues("close");
      RCLCPP_INFO(get_logger(), "Joint values for 'close':");
      for (const auto &pair : joint_values) {
          RCLCPP_INFO(get_logger(), "  %s = %f", pair.first.c_str(), pair.second);
      }

      auto active_joints = gripper_interface_->getActiveJoints();
      for (size_t i = 0; i < active_joints.size(); ++i)
        RCLCPP_INFO(this->get_logger(), "Gripper %zu: %f", i, active_joints[i]);      

      gripper_interface_->setNamedTarget("close");
      bool success = (gripper_interface_->move() == moveit::core::MoveItErrorCode::SUCCESS);

      auto result = std::make_shared<CustomActionClose::Result>();
      result->success = success;
      RCLCPP_INFO(this->get_logger(), "Goal reached successfully");

      joints = gripper_interface_->getCurrentJointValues();
      for (size_t i = 0; i < joints.size(); ++i)
        RCLCPP_INFO(this->get_logger(), "Gripper %zu: %f", i, joints[i]);

      goal_handle->succeed(result);
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

      auto joints = gripper_interface_->getCurrentJointValues();
      for (size_t i = 0; i < joints.size(); ++i)
        RCLCPP_INFO(this->get_logger(), "Gripper %zu: %f", i, joints[i]);
      
      std::map<std::string, double> joint_values = gripper_interface_->getNamedTargetValues("open");
      RCLCPP_INFO(get_logger(), "Joint values for 'open':");
      for (const auto &pair : joint_values) {
          RCLCPP_INFO(get_logger(), "  %s = %f", pair.first.c_str(), pair.second);
      }

      gripper_interface_->setNamedTarget("open");
      bool success = (gripper_interface_->move() == moveit::core::MoveItErrorCode::SUCCESS);

      auto result = std::make_shared<CustomActionOpen::Result>();
      result->success = success;
      RCLCPP_INFO(this->get_logger(), "Goal reached successfully");

      joints = gripper_interface_->getCurrentJointValues();
      for (size_t i = 0; i < joints.size(); ++i)
        RCLCPP_INFO(this->get_logger(), "Gripper %zu: %f", i, joints[i]);

      goal_handle->succeed(result);
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

      auto result = std::make_shared<CustomActionPickup::Result>();
      result->success = success;
      RCLCPP_INFO(this->get_logger(), "Goal reached successfully");

      auto current_position = move_group_interface_->getCurrentPose().pose;
      // tf2::Quaternion q_end;
      // tf2::fromMsg(quat_msg, q_end);
      double roll, pitch, yaw;
      // tf2::Matrix3x3(current_position.orientation).getRPY(roll, pitch, yaw);

      RCLCPP_INFO(this->get_logger(), "End Position is (%.2f, %.2f, %.2f)", 
        current_position.position.x, current_position.position.y, current_position.position.z);
      // RCLCPP_INFO(this->get_logger(), "End Angle is (%.2f, %.2f, %.2f)", 
        // roll, pitch, yaw);
      RCLCPP_INFO(this->get_logger(), "End Quaternion is (%.2f, %.2f, %.2f, %.2f)",
        current_position.orientation.x, current_position.orientation.y, current_position.orientation.z, current_position.orientation.w);

      goal_handle->succeed(result);
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
            // orientation_constraint.header.frame_id = move_group_interface_->getPoseReferenceFrame();
            // orientation_constraint.link_name = move_group_interface_->getEndEffectorLink();
            orientation_constraint.header.frame_id = "world"; // or base
            orientation_constraint.link_name = "fr3_hand"; // or fr3_link8

            RCLCPP_INFO(get_logger(), "EEF link: %s", move_group_interface_->getEndEffectorLink().c_str());
            RCLCPP_INFO(get_logger(), "REF frame: %s", move_group_interface_->getPoseReferenceFrame().c_str());

            auto current_pose = move_group_interface_->getCurrentPose();
            orientation_constraint.orientation = current_pose.pose.orientation;
            RCLCPP_INFO(get_logger(), "Current Pose Orientation: (%f, %f, %f, %f)",
              current_pose.pose.orientation.x, current_pose.pose.orientation.y,
              current_pose.pose.orientation.z, current_pose.pose.orientation.w
            );
            orientation_constraint.absolute_x_axis_tolerance = 0.2;
            orientation_constraint.absolute_y_axis_tolerance = 0.2;
            orientation_constraint.absolute_z_axis_tolerance = 0.2;
            orientation_constraint.weight = 1.0;

            moveit_msgs::msg::Constraints all_constraints;
            all_constraints.orientation_constraints.emplace_back(orientation_constraint);

            move_group_interface_->setPathConstraints(all_constraints);
            RCLCPP_INFO(get_logger(), "Applied orientation constraints to planning scene.");

      target.orientation.x = current_pose.pose.orientation.x;
      target.orientation.y = current_pose.pose.orientation.y;
      target.orientation.z = current_pose.pose.orientation.z; // maybe remove this
      target.orientation.w = current_pose.pose.orientation.w;

      // double roll, pitch, yaw;
      // tf2::Matrix3x3(current_pose.pose.orientation).getRPY(roll, pitch, yaw);
      RCLCPP_INFO(this->get_logger(), "Target Position is (%.2f, %.2f, %.2f)", goal->x, goal->y, goal->z);
      // RCLCPP_INFO(this->get_logger(), "Target Angle is (%.2f, %.2f, %.2f)", goal->ox, goal->oy, goal->oz);
      RCLCPP_INFO(this->get_logger(), "Target Quaternion is (%.2f, %.2f, %.2f, %.2f)", target.orientation.x, target.orientation.y, target.orientation.z, target.orientation.w);

      move_group_interface_->setPoseTarget(target);
      bool success = (move_group_interface_->move() == moveit::core::MoveItErrorCode::SUCCESS);

      auto result = std::make_shared<CustomActionMove::Result>();
      result->success = success;
      RCLCPP_INFO(this->get_logger(), "Goal reached successfully");
      goal_handle->succeed(result);

      auto current_position = move_group_interface_->getCurrentPose().pose;
      // tf2::Quaternion q_end;
      // tf2::fromMsg(quat_msg, q_end);
      double roll2, pitch2, yaw2;
      // tf2::Matrix3x3(current_position.orientation).getRPY(roll, pitch, yaw);

      RCLCPP_INFO(this->get_logger(), "End Position is (%.2f, %.2f, %.2f)", 
        current_position.position.x, current_position.position.y, current_position.position.z);
      // RCLCPP_INFO(this->get_logger(), "End Angle is (%.2f, %.2f, %.2f)", 
        // roll2, pitch2, yaw2);
      RCLCPP_INFO(this->get_logger(), "End Quaternion is (%.2f, %.2f, %.2f, %.2f)",
        current_position.orientation.x, current_position.orientation.y, current_position.orientation.z, current_position.orientation.w);


    }).detach();
  }

  // Server Callbacks for Move To Point Action
  rclcpp_action::GoalResponse handle_goal_point_(
    const rclcpp_action::GoalUUID&, std::shared_ptr<const CustomActionPoint::Goal> goal
  ) {
    RCLCPP_INFO(this->get_logger(), "Received point request (%.2f %.2f %.2f)", goal->x, goal->y, goal->z);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel_point_(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<CustomActionPoint>> goal_handle
  ) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel point");
    return rclcpp_action::CancelResponse::REJECT;
  }

  void handle_accepted_point_(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<CustomActionPoint>> goal_handle
  ) {
    std::thread([this, goal_handle]() {
      auto goal = goal_handle->get_goal();

      geometry_msgs::msg::Pose target;
      target.position.x = goal->x;
      target.position.y = goal->y;
      target.position.z = goal->z;

      move_group_interface_->setPoseTarget(target);
      bool success = (move_group_interface_->move() == moveit::core::MoveItErrorCode::SUCCESS);

      auto result = std::make_shared<CustomActionPoint::Result>();
      result->success = success;
      RCLCPP_INFO(this->get_logger(), "Goal reached successfully");
      goal_handle->succeed(result);
    }).detach();
  }

  void clicked_point_subscriber_callback_(StampedPoint::UniquePtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received Clicked Point: (%f, %f, %f)", msg->point.x, msg->point.y, msg->point.z);

    // create a goal message
    auto goal_msg = CustomActionPoint::Goal();
    goal_msg.x = 0.5; goal_msg.y = 0.0; goal_msg.z = 0.3;

    // sets callbacks for action
    auto send_goal_options = rclcpp_action::Client<CustomActionPoint>::SendGoalOptions();
    send_goal_options.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<CustomActionPoint>::WrappedResult & result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
        } else {
          RCLCPP_ERROR(this->get_logger(), "Goal failed with code: %d", (int)result.code);
        }
      };

    // sends action
    moveit_client_->async_send_goal(goal_msg, send_goal_options);
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

  void goal_pose_subscriber_callback_(StampedPose::UniquePtr msg) {
    RCLCPP_DEBUG(this->get_logger(), 
      "Goal Pose: (%f, %f, %f | %f, %f, %f, %f)", 
      msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
      msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w  
    );
  }

  void goal_pose_publisher_callback_() {
    auto message = StampedPose{};
      
    // Fill header
    message.header.stamp = rclcpp::Clock().now();
    message.header.frame_id = "base_link";

    // Fill pose
    message.pose.position.x = 1.0;
    message.pose.position.y = 2.0;
    message.pose.position.z = 3.0;

    // Orientation as a quaternion (no rotation)
    message.pose.orientation.x = 0.0;
    message.pose.orientation.y = 0.0;
    message.pose.orientation.z = 0.0;
    message.pose.orientation.w = 1.0;

    RCLCPP_DEBUG(this->get_logger(), "Publishing a stamped pose");
    this->goal_pose_pub_->publish(message);
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