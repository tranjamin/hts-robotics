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
#include "hts_robotics/action/move_to_point.hpp"

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

  hts_node():Node("hts_node"), 
  count_(0)
  
  {
    RCLCPP_INFO(this->get_logger(), "Initialising the HTS Robotics Node");

    // create a subscriber to take in a clicked point and move the robot
    RCLCPP_INFO(this->get_logger(), "Creating clicked point subscriber...");
    clicked_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>("/clicked_point", 10,
      [this](geometry_msgs::msg::PointStamped::UniquePtr msg) -> void {
        RCLCPP_INFO(this->get_logger(), "Clicked Point: (%f, %f, %f)", msg->point.x, msg->point.y, msg->point.z);

        // creates a goal message
        auto goal_msg = hts_robotics::action::MoveToPoint::Goal();
        goal_msg.x = 0.5;
        goal_msg.y = 0.0;
        goal_msg.z = 0.3;

        // sets callbacks for action
        auto send_goal_options = rclcpp_action::Client<hts_robotics::action::MoveToPoint>::SendGoalOptions();
        send_goal_options.result_callback =
          [this](const rclcpp_action::ClientGoalHandle<hts_robotics::action::MoveToPoint>::WrappedResult & result) {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
              RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
            } else {
              RCLCPP_ERROR(this->get_logger(), "Goal failed with code: %d", (int)result.code);
            }
          };

        // sends action
        moveit_client_->async_send_goal(goal_msg, send_goal_options);
      });

    // create a subscriber to read in the goal pose and log info
    RCLCPP_INFO(this->get_logger(), "Creating goal pose subscriber...");
    goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 10,
      [this](geometry_msgs::msg::PoseStamped::UniquePtr msg) -> void {
        RCLCPP_INFO(this->get_logger(), "Goal Pose: (%f, %f, %f | %f, %f, %f, %f)", 
          msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
          msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w  
        );
      }
    );

    // create a subscriber to read in the joint states and log info
    RCLCPP_INFO(this->get_logger(), "Creating joint states subscriber...");
    joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10,
      [this](sensor_msgs::msg::JointState::UniquePtr msg) -> void {
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
    );

    // create a publisher for the goal pose
    RCLCPP_INFO(this->get_logger(), "Creating goal pose publisher...");
    goal_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
    timer_ = this->create_wall_timer(500ms, [this]() -> void {

      auto message = geometry_msgs::msg::PoseStamped{};
      
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
    });

    // create moveit server
    RCLCPP_INFO(this->get_logger(), "Creating MoveIt2 Server...");
    moveit_server_ = rclcpp_action::create_server<hts_robotics::action::MoveToPoint>(
      this, "hts_moveit_action",
      std::bind(&hts_node::handle_goal_, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&hts_node::handle_cancel_, this, std::placeholders::_1),
      std::bind(&hts_node::handle_accepted_, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "Creating MoveIt2 Client...");
    moveit_client_ = rclcpp_action::create_client<hts_robotics::action::MoveToPoint>(this, "hts_moveit_action");    
    if (!moveit_client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to connect to action server");
      rclcpp::shutdown();
      return;
    }

    RCLCPP_WARN(this->get_logger(), "Finished constructing node...");
    // RCLCPP_INFO(this->get_logger(), "Planning scene frame: %s", move_group_interface_.getPlanningFrame().c_str());
  
  }

  void init() {
    RCLCPP_INFO(this->get_logger(), "Starting init...");
    
    move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), "fr3_arm");

    planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

    RCLCPP_INFO(get_logger(), "MoveGroup and PlanningSceneInterface initialized. Planning frame: %s",
                move_group_interface_->getPlanningFrame().c_str());

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
    planning_scene_interface_->applyCollisionObject(collision_object);
    RCLCPP_INFO(get_logger(), "Applied collision object 'box1' to planning scene.");
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub_;

  rclcpp_action::Client<hts_robotics::action::MoveToPoint>::SharedPtr moveit_client_;
  rclcpp_action::Server<hts_robotics::action::MoveToPoint>::SharedPtr moveit_server_;

  rclcpp_action::GoalResponse handle_goal_(
    const rclcpp_action::GoalUUID&, std::shared_ptr<const hts_robotics::action::MoveToPoint::Goal> goal
  ) {
    RCLCPP_INFO(this->get_logger(), "Received goal request (%.2f %.2f %.2f)", goal->x, goal->y, goal->z);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel_(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<hts_robotics::action::MoveToPoint>> goal_handle
  ) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::REJECT;
  }

  void handle_accepted_(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<hts_robotics::action::MoveToPoint>> goal_handle
  ) {
    std::thread([this, goal_handle]() {
      auto goal = goal_handle->get_goal();

      geometry_msgs::msg::Pose target;
      target.position.x = goal->x;
      target.position.y = goal->y;
      target.position.z = goal->z;

      move_group_interface_->setPoseTarget(target);
      bool success = (move_group_interface_->move() == moveit::core::MoveItErrorCode::SUCCESS);

      auto result = std::make_shared<hts_robotics::action::MoveToPoint::Result>();
      result->success = success;
      RCLCPP_INFO(this->get_logger(), "Goal reached successfully");
      goal_handle->succeed(result);
    }).detach();
  
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