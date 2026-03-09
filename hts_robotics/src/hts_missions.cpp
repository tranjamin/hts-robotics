#include <chrono>
#include <memory>
#include <string>

// includes
#include "rclcpp/rclcpp.hpp"
#include <rclcpp_action/rclcpp_action.hpp>
#include "std_msgs/msg/string.hpp"

#include "hts_msgs/action/move_target.hpp"
#include "hts_msgs/action/pick_up_target.hpp"
#include "hts_msgs/action/gripper_open.hpp"
#include "hts_msgs/action/gripper_close.hpp"
#include "hts_msgs/action/grasp_object.hpp"
#include "hts_msgs/action/compute_grasp_validity.hpp"
#include "hts_msgs/action/request_grasp.hpp"
#include "hts_msgs/srv/get_object_position.hpp"

class hts_missions : public rclcpp::Node {
  public:
    // type definitions
    using CustomActionPickup = hts_msgs::action::PickUpTarget;
    using CustomActionMove = hts_msgs::action::MoveTarget;

    using CustomActionOpen = hts_msgs::action::GripperOpen;
    using CustomActionClose = hts_msgs::action::GripperClose;

    using CustomActionComputeGraspValidity = hts_msgs::action::ComputeGraspValidity;
    using CustomActionGraspObject = hts_msgs::action::GraspObject;
    using CustomActionRequestGrasp = hts_msgs::action::RequestGrasp;
    using CustomServiceObjectPosition = hts_msgs::srv::GetObjectPosition;

    hts_missions():Node("hts_missions") {
      RCLCPP_INFO(this->get_logger(), "Constructing HTS Missions Node...");
      
      grasp_object_server_ = rclcpp_action::create_server<CustomActionGraspObject>(
        this, "grasp_object",
        std::bind(&hts_missions::handle_goal_grasp_object_, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&hts_missions::handle_cancel_grasp_object_, this, std::placeholders::_1),
        std::bind(&hts_missions::handle_accepted_grasp_object_, this, std::placeholders::_1)
      );

      grasp_request_client_ = rclcpp_action::create_client<hts_msgs::action::RequestGrasp>(this, "request_grasp");
      
      pickup_client_ = rclcpp_action::create_client<CustomActionPickup>(this, "hts_pickup_action");
      move_client_ = rclcpp_action::create_client<CustomActionMove>(this, "hts_move_action");
      open_client_ = rclcpp_action::create_client<CustomActionOpen>(this, "gripper_open");
      close_client_ = rclcpp_action::create_client<CustomActionClose>(this, "gripper_close");

      object_position_client_ = this->create_client<hts_msgs::srv::GetObjectPosition>("get_object_position");

      RCLCPP_INFO(this->get_logger(), "Constructed HTS Missions Node.");
    }

    void init() {
      RCLCPP_INFO(this->get_logger(), "Initialising HTS Missions Node...");
      grasp_request_client_->wait_for_action_server();
      object_position_client_->wait_for_service();
      RCLCPP_INFO(this->get_logger(), "Initialised HTS Missions Node.");
    }

  private:

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
        auto result = std::make_shared<CustomActionGraspObject::Result>();
        auto feedback = std::make_shared<CustomActionGraspObject::Feedback>();

        auto position_request = std::make_shared<CustomServiceObjectPosition::Request>();
        int object_id = goal_handle->get_goal()->object_id;
        position_request->object_id = object_id;

        feedback->progress = "Requesting Object Position...";
        goal_handle->publish_feedback(feedback);
        std::shared_future<CustomServiceObjectPosition::Response::SharedPtr> position_future = object_position_client_->async_send_request(position_request);
        CustomServiceObjectPosition::Response::SharedPtr position_response = position_future.get();
        feedback->progress = "Received Object Position.";
        goal_handle->publish_feedback(feedback);

        if (!position_response->success) {
          RCLCPP_ERROR(this->get_logger(), "Failed to identify object pose");
          result->success = false;
          result->message = "Failed to identify object pose";
          goal_handle->abort(result);
          return;
        }

        // get the grasp for the target object
        auto grasp_request = CustomActionRequestGrasp::Goal();
        grasp_request.id = object_id;
        grasp_request.x = position_response.get()->x;
        grasp_request.y = position_response.get()->y;
        grasp_request.z = position_response.get()->z;
        grasp_request.goal_x = goal_handle->get_goal()->x;
        grasp_request.goal_y = goal_handle->get_goal()->y;
        grasp_request.goal_z = goal_handle->get_goal()->z;

        auto pickup_goal = CustomActionPickup::Goal();

        auto open_send_goal_options = rclcpp_action::Client<CustomActionOpen>::SendGoalOptions();
        open_send_goal_options.result_callback =
          [this, goal_handle, result, feedback](const rclcpp_action::ClientGoalHandle<CustomActionOpen>::WrappedResult &r) {
            if (r.code != rclcpp_action::ResultCode::SUCCEEDED) {
              RCLCPP_ERROR(this->get_logger(), "Gripper failed to open with code: %d", (int)r.code);
              result->success = false;
              result->message = "Gripper failed to open";
              goal_handle->abort(result);
            } else {
              feedback->progress = "Gripper opened on target";
              goal_handle->publish_feedback(feedback);
              result->success = true;
              result->message = "Finished execution";
              goal_handle->succeed(result);
            }
          };    


        auto move_send_goal_options = rclcpp_action::Client<CustomActionMove>::SendGoalOptions();
        move_send_goal_options.result_callback =
          [this, open_send_goal_options, goal_handle, result, feedback, object_id](const rclcpp_action::ClientGoalHandle<CustomActionMove>::WrappedResult &r) {
            if (r.code != rclcpp_action::ResultCode::SUCCEEDED) {
              RCLCPP_ERROR(this->get_logger(), "MoveIt failed to move with code: %d", (int)r.code);
              result->success = false;
              result->message = "MoveIt failed to move";
              goal_handle->abort(result);
            } else {
              feedback->progress = "MoveIt moved target";
              goal_handle->publish_feedback(feedback);

              auto open_goal = CustomActionOpen::Goal();
              open_goal.target_id = object_id;
              open_client_->async_send_goal(open_goal, open_send_goal_options);
            }
          };

        auto close_send_goal_options = rclcpp_action::Client<CustomActionClose>::SendGoalOptions();
        close_send_goal_options.result_callback =
          [this, move_send_goal_options, goal_handle, result, feedback](const rclcpp_action::ClientGoalHandle<CustomActionClose>::WrappedResult &r) {
            if (r.code != rclcpp_action::ResultCode::SUCCEEDED) {
              RCLCPP_ERROR(this->get_logger(), "Gripper failed to close with code: %d", (int)r.code);
              result->success = false;
              result->message = "Gripper failed to close";
              goal_handle->abort(result);
            } else {
              feedback->progress = "Gripper closed on target";
              goal_handle->publish_feedback(feedback);

              auto move_goal = CustomActionMove::Goal();
              move_goal.x = goal_handle->get_goal()->x;
              move_goal.y = goal_handle->get_goal()->y;
              move_goal.z = goal_handle->get_goal()->z;

              move_client_->async_send_goal(move_goal, move_send_goal_options);
            }
          };

        auto pickup_send_goal_options = rclcpp_action::Client<CustomActionPickup>::SendGoalOptions();
        pickup_send_goal_options.result_callback =
          [this, close_send_goal_options, goal_handle, result, feedback, object_id](const rclcpp_action::ClientGoalHandle<CustomActionPickup>::WrappedResult &r) {
            if (r.code != rclcpp_action::ResultCode::SUCCEEDED) {
              RCLCPP_ERROR(this->get_logger(), "Movet failed to move to target with code: %d", (int)r.code);
              result->success = false;
              result->message = "MoveIt failed to move to target";
              goal_handle->abort(result);
            } else {   
              feedback->progress = "MoveIt moved to target";
              goal_handle->publish_feedback(feedback);

              auto close_goal = CustomActionClose::Goal();
              close_goal.target_id = object_id;
              close_client_->async_send_goal(close_goal, close_send_goal_options);
            }
          };

        auto first_open_send_goal_options = rclcpp_action::Client<CustomActionOpen>::SendGoalOptions();
        first_open_send_goal_options.result_callback =
          [this, pickup_send_goal_options, goal_handle, result, feedback, pickup_goal](const rclcpp_action::ClientGoalHandle<CustomActionOpen>::WrappedResult &r) {
            if (r.code != rclcpp_action::ResultCode::SUCCEEDED) {
              RCLCPP_ERROR(this->get_logger(), "Gripper failed to open with code: %d", (int)r.code);
              result->success = false;
              result->message = "Gripper failed to open";
              goal_handle->abort(result);
            } else {
              feedback->progress = "Gripper opened on target";
              goal_handle->publish_feedback(feedback);
              pickup_client_->async_send_goal(pickup_goal, pickup_send_goal_options);
            }
          };
          
        auto grasp_object_send_goal_options = rclcpp_action::Client<hts_msgs::action::RequestGrasp>::SendGoalOptions();
        grasp_object_send_goal_options.result_callback =
          [this, first_open_send_goal_options, goal_handle, result, feedback, &pickup_goal, object_id](const rclcpp_action::ClientGoalHandle<hts_msgs::action::RequestGrasp>::WrappedResult &r) {
            if (r.code != rclcpp_action::ResultCode::SUCCEEDED) {
              RCLCPP_ERROR(this->get_logger(), "Anygrasp failed to identify pose");
              result->success = false;
              result->message = "Anygrasp failed to identify pose";
              goal_handle->abort(result);
              return;
            } else {
              geometry_msgs::msg::Pose grasp_pose = r.result->grasp_pose;

              char buf[150];
              std::snprintf(buf, sizeof(buf),
                "AnyGrasp Found a Grasp of (%.2f, %.2f, %.2f) (%.2f, %.2f, %.2f, %.2f)", 
                grasp_pose.position.x, grasp_pose.position.y, grasp_pose.position.z,
                grasp_pose.orientation.x, grasp_pose.orientation.y, grasp_pose.orientation.z, grasp_pose.orientation.w
              );

              feedback->progress = std::string(buf);
              goal_handle->publish_feedback(feedback);

              pickup_goal.pose = grasp_pose;

              auto first_open_goal = CustomActionOpen::Goal();
              first_open_goal.target_id = object_id;
              open_client_->async_send_goal(first_open_goal, first_open_send_goal_options);
            }
          };
        grasp_object_send_goal_options.feedback_callback =
          [this, feedback, goal_handle](const rclcpp_action::ClientGoalHandle<hts_msgs::action::RequestGrasp>::SharedPtr, const std::shared_ptr<const CustomActionRequestGrasp::Feedback> f) {
            char buf[150];
            std::snprintf(buf, sizeof(buf), "Grasping Progress: %s", f->progress.c_str());
            feedback->progress = std::string(buf);
            goal_handle->publish_feedback(feedback);
          };

        // sends action
        feedback->progress = "Sending Grasp Request...";
        goal_handle->publish_feedback(feedback);
        grasp_request_client_->async_send_goal(grasp_request, grasp_object_send_goal_options);

      }).detach();
    }
  
    rclcpp_action::Server<CustomActionGraspObject>::SharedPtr grasp_object_server_;
    rclcpp::Client<hts_msgs::srv::GetObjectPosition>::SharedPtr object_position_client_;

    // for the top-level actions
    rclcpp_action::Client<hts_msgs::action::RequestGrasp>::SharedPtr grasp_request_client_;
    rclcpp_action::Client<CustomActionPickup>::SharedPtr pickup_client_;
    rclcpp_action::Client<CustomActionMove>::SharedPtr move_client_;
    rclcpp_action::Client<CustomActionOpen>::SharedPtr open_client_;
    rclcpp_action::Client<CustomActionClose>::SharedPtr close_client_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto mission_node = std::make_shared<hts_missions>();
  mission_node->init();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(mission_node);
  executor.spin();
  // rclcpp::spin(mission_node);
  rclcpp::shutdown();
  return 0;
}