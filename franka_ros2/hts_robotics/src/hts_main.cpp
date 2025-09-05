#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;


/**
 * Contains:
 *  
 */
class hts_node : public rclcpp::Node {
public:
  hts_node():Node("hts_node"), count_(0) {

    clicked_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>("/clicked_point", 10,
      [this](geometry_msgs::msg::PointStamped::UniquePtr msg) -> void {
        RCLCPP_INFO(this->get_logger(), "Clicked Point: (%f, %f, %f)", msg->point.x, msg->point.y, msg->point.z);
      }
    );

    goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 10,
      [this](geometry_msgs::msg::PoseStamped::UniquePtr msg) -> void {
        RCLCPP_INFO(this->get_logger(), "Goal Pose: (%f, %f, %f | %f, %f, %f, %f)", 
          msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
          msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w  
        );
      }
    );

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
        // RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
      }
    );

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

      // RCLCPP_INFO(this->get_logger(), "Publishing a stamped pose");
      // this->goal_pose_pub_->publish(message);
    });

  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub_;
  
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hts_node>());
  rclcpp::shutdown();
  return 0;
}