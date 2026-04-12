// hts_plugins.cpp
#include <hts_plugins/hts_constraint_sampler.hpp>
#include <moveit/constraint_samplers/constraint_sampler.hpp>
#include <moveit/constraint_samplers/default_constraint_samplers.hpp>
#include <moveit/constraint_samplers/constraint_sampler_allocator.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <moveit/utils/logger.hpp>
#include <geometry_msgs/msg/pose.hpp>

// HELPER FUNCTIONS
rclcpp::Logger getLogger() {
  return rclcpp::get_logger("hts_plugin");
}

void printConstraints(const moveit_msgs::msg::Constraints& c) {
    // --- Position Constraints ---
    for (size_t i = 0; i < c.position_constraints.size(); ++i)
    {
        const auto& pc = c.position_constraints[i];
        RCLCPP_INFO(getLogger(), "    [PositionConstraint %zu] link: %s", i, pc.link_name.c_str());
        RCLCPP_INFO(getLogger(), "      frame: %s", pc.header.frame_id.c_str());
        RCLCPP_INFO(getLogger(), "      target point offset: (%f, %f, %f)",
                    pc.target_point_offset.x,
                    pc.target_point_offset.y,
                    pc.target_point_offset.z);
        RCLCPP_INFO(getLogger(), "      tolerance: x=%f y=%f z=%f",
                    pc.constraint_region.primitives[0].dimensions[0],
                    pc.constraint_region.primitives[0].dimensions[0],
                    pc.constraint_region.primitives[0].dimensions[0]);
    }

    // --- Orientation Constraints ---
    for (size_t i = 0; i < c.orientation_constraints.size(); ++i)
    {
        const auto& oc = c.orientation_constraints[i];
        RCLCPP_INFO(getLogger(), "    [OrientationConstraint %zu] link: %s", i, oc.link_name.c_str());
        RCLCPP_INFO(getLogger(), "      frame: %s", oc.header.frame_id.c_str());
        RCLCPP_INFO(getLogger(), "      orientation: (%f, %f, %f, %f)",
                    oc.orientation.x, oc.orientation.y, oc.orientation.z, oc.orientation.w);
        RCLCPP_INFO(getLogger(), "      tolerances: x=%f y=%f z=%f, weight=%f",
                    oc.absolute_x_axis_tolerance, oc.absolute_y_axis_tolerance,
                    oc.absolute_z_axis_tolerance, oc.weight);
    }

    // --- Joint Constraints ---
    for (size_t i = 0; i < c.joint_constraints.size(); ++i)
    {
        const auto& jc = c.joint_constraints[i];
        RCLCPP_INFO(getLogger(), "    [JointConstraint %zu] joint: %s", i, jc.joint_name.c_str());
        RCLCPP_INFO(getLogger(), "      position: %f, tolerance: %f, weight: %f",
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

void log_pose(const geometry_msgs::msg::Pose &pose, const char* descriptor="") {
    RCLCPP_INFO(getLogger(), "%s Position is (%.5f, %.5f, %.5f)", descriptor, pose.position.x, pose.position.y, pose.position.z);
    RCLCPP_INFO(getLogger(), "%s Quaternion is (%.5f, %.5f, %.5f, %.5f)", descriptor, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
}

void samplingIkCallbackFnAdapter(moveit::core::RobotState* state, const moveit::core::JointModelGroup* jmg,
                                 const moveit::core::GroupStateValidityCallbackFn& constraint,
                                 const std::vector<double>& ik_sol, moveit_msgs::msg::MoveItErrorCodes& error_code) {
  const std::vector<size_t>& bij = jmg->getKinematicsSolverJointBijection();
  std::vector<double> solution(bij.size());
  for (std::size_t i = 0; i < bij.size(); ++i)
    solution[i] = ik_sol[bij[i]];
  if (constraint(state, jmg, &solution[0])) {
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  } else {
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
  }
}

// CONSTRAINT SAMPLER FUNCTIONS
hts_plugins::HTSIKConstraintSampler::HTSIKConstraintSampler(
    const std::shared_ptr<const planning_scene::PlanningScene>& scene,
    const std::string& group_name)
    : constraint_samplers::IKConstraintSampler(scene, group_name) {
    RCLCPP_INFO(getLogger(), "HTSIKConstraintSampler constructor called for group %s", group_name.c_str());
}

const std::string& hts_plugins::HTSIKConstraintSampler::getName() const {
    static const std::string SAMPLER_NAME = "HTSIKConstraintSampler";
    return SAMPLER_NAME;
}

bool hts_plugins::HTSIKConstraintSampler::sample(moveit::core::RobotState &state, const moveit::core::RobotState &reference_state, unsigned int max_attempts) {
    return hts_plugins::HTSIKConstraintSampler::sampleHelper(state, reference_state, max_attempts);
}

bool hts_plugins::HTSIKConstraintSampler::sampleHelper(moveit::core::RobotState& state, const moveit::core::RobotState& reference_state, unsigned int max_attempts) {
    if (!is_valid_) {
        RCLCPP_WARN(getLogger(), "HTSIKConstraintSampler not configured, won't sample");
        return false;
    }

    kinematics::KinematicsBase::IKCallbackFn adapted_ik_validity_callback;
    if (group_state_validity_callback_) {
        adapted_ik_validity_callback = [this, state_ptr = &state](const geometry_msgs::msg::Pose&, const std::vector<double>& joints, moveit_msgs::msg::MoveItErrorCodes& error_code) {
        return samplingIkCallbackFnAdapter(state_ptr, jmg_, group_state_validity_callback_, joints, error_code);
        };
    }

    for (unsigned int a = 0; a < max_attempts; ++a) {
        // sample a point in the constraint region
        Eigen::Vector3d point;
        Eigen::Quaterniond quat;  // quat is normalized by contract
        if (!samplePose(point, quat, reference_state, max_attempts)) {
            if (verbose_) RCLCPP_INFO(getLogger(), "IK constraint sampler was unable to produce a pose to run IK for");
            return false;
        }

        // we now have the transform we wish to perform IK for, in the planning frame
        if (transform_ik_) {
            // we need to convert this transform to the frame expected by the IK solver
            // both the planning frame and the frame for the IK are assumed to be robot links
            Eigen::Isometry3d ikq(Eigen::Translation3d(point) * quat);  // valid isometry by construction
            // getFrameTransform() returns a valid isometry by contract
            ikq = reference_state.getFrameTransform(ik_frame_).inverse() * ikq;  // valid isometry * valid isometry
            point = ikq.translation();
            quat = Eigen::Quaterniond(ikq.linear());  // ikq is isometry, so quat is normalized
        }

        if (need_eef_to_ik_tip_transform_) {
            // After sampling the pose needs to be transformed to the ik chain tip
            Eigen::Isometry3d ikq(Eigen::Translation3d(point) * quat);  // valid isometry by construction
            ikq = ikq * eef_to_ik_tip_transform_;  // eef_to_ik_tip_transform_ is valid isometry (checked in loadIKSolver())
            point = ikq.translation();
            quat = Eigen::Quaterniond(ikq.linear());  // ikq is isometry, so quat is normalized
        }

        geometry_msgs::msg::Pose ik_query;
        ik_query.position.x = point.x();
        ik_query.position.y = point.y();
        ik_query.position.z = point.z();
        ik_query.orientation.x = quat.x();
        ik_query.orientation.y = quat.y();
        ik_query.orientation.z = quat.z();
        ik_query.orientation.w = quat.w();

        if (constraint_samplers::IKConstraintSampler::callIK(ik_query, adapted_ik_validity_callback, ik_timeout_, state, a == 0))
        return true;
    }
  return false;
}

bool hts_plugins::HTSIKConstraintSampler::configure(const moveit_msgs::msg::Constraints &constr) {
    RCLCPP_INFO(getLogger(), "Configuring HTSIKConstraintSampler...");
    bool ret = constraint_samplers::IKConstraintSampler::configure(constr);
    
    if (!ret) RCLCPP_ERROR(getLogger(), "Configure failed.");

    return ret;
}

bool hts_plugins::HTSIKConstraintSampler::samplePose(Eigen::Vector3d& pos, Eigen::Quaterniond& quat, const moveit::core::RobotState& ks, unsigned int max_attempts) {
    // RCLCPP_INFO(getLogger(), "HTSIK Sampling Pose from HTSIKConstraintSampler...");

    if (ks.dirtyLinkTransforms()) {
        // samplePose below requires accurate transforms
        RCLCPP_ERROR(getLogger(), "HTSIK IKConstraintSampler received dirty robot state, but valid transforms are required. Failing.");
        return false;
    }

    const std::vector<bodies::BodyPtr>& b = sampling_pose_.position_constraint_->getConstraintRegions();
    if (!b.empty()) {
      bool found = false;
      std::size_t k = random_number_generator_.uniformInteger(0, b.size() - 1);
      for (std::size_t i = 0; i < b.size(); ++i) {
        if (b[(i + k) % b.size()]->samplePointInside(random_number_generator_, max_attempts, pos)) {
          found = true;
          break;
        }
      }
      if (!found) {
        RCLCPP_ERROR(getLogger(), "Unable to sample a point inside the constraint region");
        return false;
      }
    }

    // sample a rotation matrix within the allowed bounds
    double angle_x = 2.0 * (random_number_generator_.uniform01() - 0.5) * (sampling_pose_.orientation_constraint_->getXAxisTolerance() - std::numeric_limits<double>::epsilon());
    double angle_y = 2.0 * (random_number_generator_.uniform01() - 0.5) * (sampling_pose_.orientation_constraint_->getYAxisTolerance() - std::numeric_limits<double>::epsilon());
    double angle_z = 2.0 * (random_number_generator_.uniform01() - 0.5) * (sampling_pose_.orientation_constraint_->getZAxisTolerance() - std::numeric_limits<double>::epsilon());
    
    // RCLCPP_INFO(getLogger(), "HTSIK Assuming ROTATION_VECTOR parametrisation");
    // RCLCPP_INFO(getLogger(), "HTSIK Reference Frame: %s", sampling_pose_.orientation_constraint_->getReferenceFrame().c_str());

    Eigen::Isometry3d diff;
    Eigen::Vector3d rotation_vector(angle_x, angle_y, angle_z);
    
    // convert rotation vector from frame_id to target frame
    rotation_vector = sampling_pose_.orientation_constraint_->getDesiredRotationMatrixInRefFrame().transpose() * rotation_vector;
    diff = Eigen::Isometry3d(Eigen::AngleAxisd(rotation_vector.norm(), rotation_vector.normalized()));

    // get the desired orientation quaternion
    quat = Eigen::Quaterniond(sampling_pose_.orientation_constraint_->getDesiredRotationMatrix() * diff.linear());
    
    // if there is an offset, we need to undo the induced rotation in the sampled transform origin (point)
    if (sampling_pose_.position_constraint_ && sampling_pose_.position_constraint_->hasLinkOffset()) {
        // the rotation matrix that corresponds to the desired orientation
        pos = pos - quat * sampling_pose_.position_constraint_->getLinkOffset();
    }

    // now we should validate the axis alignment
    // RCLCPP_INFO(getLogger(), "HTSIK Calculating Axis Alignment...");
    Eigen::Vector3d translated_x_axis = (sampling_pose_.orientation_constraint_->getDesiredRotationMatrix() * diff.linear()) * Eigen::Vector3d(1, 0, 0);
    Eigen::Vector3d reference_axis = sampling_pose_.orientation_constraint_->getDesiredRotationMatrix() * Eigen::Vector3d(1, 0, 0);
    // RCLCPP_INFO(getLogger(), "Translated X Axis is: %.5f %.5f %.5f", translated_x_axis.x(), translated_x_axis.y(), translated_x_axis.z());
    // RCLCPP_INFO(getLogger(), "Original X Axis is: %.5f %.5f %.5f", reference_axis.x(), reference_axis.y(), reference_axis.z());

    double alignment = translated_x_axis.dot(reference_axis);
    // RCLCPP_DEBUG(getLogger(), "HTSIK Axis Alignment is %f, angle errors are %f, %f, %f", alignment, angle_x, angle_y, angle_z);
    
    geometry_msgs::msg::Pose pose;

    pose.position.x = pos.x();
    pose.position.y = pos.y();
    pose.position.z = pos.z();

    pose.orientation.x = quat.x();
    pose.orientation.y = quat.y();
    pose.orientation.z = quat.z();
    pose.orientation.w = quat.w();

    if (alignment < 0.95) {
        // RCLCPP_INFO(getLogger(), "Alignment is not good... failing.");
        return false;
    }

    log_pose(pose, "HTSIK Received Sample");

    return true;
}

// FOR CONSTRAINT ALLOCATOR
hts_plugins::HTSIKConstraintSamplerAllocator::HTSIKConstraintSamplerAllocator() {
    RCLCPP_INFO(getLogger(), "Sampler Allocator Constructor");
}

std::shared_ptr<constraint_samplers::ConstraintSampler> hts_plugins::HTSIKConstraintSamplerAllocator::alloc(
    const planning_scene::PlanningSceneConstPtr &scene, 
    const std::string &group_name, 
    const moveit_msgs::msg::Constraints &constr
) {
    auto sampler = std::make_shared<hts_plugins::HTSIKConstraintSampler>(scene, group_name);
    sampler->configure(constr);
    RCLCPP_INFO(getLogger(), "HTSIKConstraintSampler allocator called for group %s", group_name.c_str());
    return sampler;
}

bool hts_plugins::HTSIKConstraintSamplerAllocator::canService(
    const planning_scene::PlanningSceneConstPtr &scene, 
    const std::string &group_name, 
    const moveit_msgs::msg::Constraints &constr
) const {
    RCLCPP_INFO(getLogger(), "HTSIKConstraintSampler canService called for group %s", group_name.c_str());
    printConstraints(constr);

    if (
        constr.orientation_constraints.size() == 1 && (
            constr.orientation_constraints[0].absolute_x_axis_tolerance > 2.0 ||
            constr.orientation_constraints[0].absolute_y_axis_tolerance > 2.0 ||
            constr.orientation_constraints[0].absolute_z_axis_tolerance > 2.0)
    ) {

        RCLCPP_INFO(getLogger(), "HTSIKConstraintSampler compatible due to orientation constraint");
        return true;
    }

    if (
        constr.orientation_constraints.size() == 2 && constr.position_constraints.size() && (
            constr.orientation_constraints[0].absolute_x_axis_tolerance > 2.0 ||
            constr.orientation_constraints[0].absolute_y_axis_tolerance > 2.0 ||
            constr.orientation_constraints[0].absolute_z_axis_tolerance > 2.0 ||
            constr.orientation_constraints[1].absolute_x_axis_tolerance > 2.0 ||
            constr.orientation_constraints[1].absolute_y_axis_tolerance > 2.0 ||
            constr.orientation_constraints[1].absolute_z_axis_tolerance > 2.0
        )
    ) {

        RCLCPP_INFO(getLogger(), "HTSIKConstraintSampler compatible due to two orientation constraints plus position constraint");
        return true;
    }



    if (constr.name == "hts_constraint") {
        RCLCPP_INFO(getLogger(), "HTSIKConstraintSampler compatible due to constraint name");
        return true;
    }

    RCLCPP_INFO(getLogger(), "HTSIKConstraintSampler not compatible");
    return false;
}

PLUGINLIB_EXPORT_CLASS(
    hts_plugins::HTSIKConstraintSamplerAllocator, 
    constraint_samplers::ConstraintSamplerAllocator
)