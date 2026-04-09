#include "hts_node.hpp"

#include <string>

#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>

#include <moveit/collision_detection/collision_common.hpp>
#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/msg/mesh.hpp>
#include "geometry_msgs/msg/pose.hpp"

#include <tf2_msgs/msg/tf_message.hpp>

void hts_node::load_target_objects() {
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
    RCLCPP_DEBUG(this->get_logger(), "Found an Object with ID %d", obj_id);

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

void hts_node::add_ground_collision() {
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
    RCLCPP_INFO(this->get_logger(), "Applied collision object 'ground' to planning scene.");
}

void hts_node::register_grasped_object(std::string object_name) {
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

void hts_node::deregister_grasped_object(std::string object_name) {
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

void hts_node::gazebo_scene_subscriber_callback_(tf2_msgs::msg::TFMessage::UniquePtr msg) {
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