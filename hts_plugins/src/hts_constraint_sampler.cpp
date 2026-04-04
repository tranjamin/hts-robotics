// hts_plugins.cpp
#include <moveit/constraint_samplers/constraint_sampler.hpp>
#include <moveit/constraint_samplers/default_constraint_samplers.hpp>
#include <moveit/constraint_samplers/constraint_sampler_allocator.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <moveit/utils/logger.hpp>

namespace hts_plugins
{
    class HTSIKConstraintSampler : public constraint_samplers::IKConstraintSampler {
        public:
            HTSIKConstraintSampler(
                const std::shared_ptr<const planning_scene::PlanningScene>& scene,
                const std::string& group_name)
                : constraint_samplers::IKConstraintSampler(scene, group_name)
            {
                RCLCPP_INFO(rclcpp::get_logger("hts_plugins"), "HTSIKConstraintSampler constructor called for group %s", group_name.c_str());
            }

            const std::string& getName() const override {
                static const std::string SAMPLER_NAME = "HTSIKConstraintSampler";
                return SAMPLER_NAME;
            }
    };

    class HTSIKConstraintSamplerAllocator : public constraint_samplers::ConstraintSamplerAllocator {
        public:
            HTSIKConstraintSamplerAllocator() {
                RCLCPP_INFO(rclcpp::get_logger("hts_plugins"), "Sampler Allocator Constructor");
            }

            std::shared_ptr<constraint_samplers::ConstraintSampler> alloc(
                const planning_scene::PlanningSceneConstPtr &scene, 
                const std::string &group_name, 
                const moveit_msgs::msg::Constraints &constr
            ) override {
                auto sampler = std::make_shared<hts_plugins::HTSIKConstraintSampler>(scene, group_name);
                sampler->configure(constr);
                RCLCPP_INFO(rclcpp::get_logger("hts_plugins"), "HTSIKConstraintSampler allocator called for group %s", group_name.c_str());
                return sampler;
            }

            bool canService(
                const planning_scene::PlanningSceneConstPtr &scene, 
                const std::string &group_name, 
                const moveit_msgs::msg::Constraints &constr
            ) const override {
                RCLCPP_INFO(rclcpp::get_logger("hts_plugins"), "HTSIKConstraintSampler canService called for group %s", group_name.c_str());

                if (
                    constr.orientation_constraints.size() == 1 && constr.orientation_constraints[0].absolute_z_axis_tolerance > 2.0
                ) {

                    RCLCPP_INFO(rclcpp::get_logger("hts_plugins"), "HTSIKConstraintSampler compatible due to orientation constraint");
                    return true;
                }


                if (constr.name == "hts_constraint") {
                    RCLCPP_INFO(rclcpp::get_logger("hts_plugins"), "HTSIKConstraintSampler compatible due to constraint name");
                    return true;
                }

                RCLCPP_INFO(rclcpp::get_logger("hts_plugins"), "HTSIKConstraintSampler not compatible");
                return false;
            }
    };

}  // namespace hts_plugins

PLUGINLIB_EXPORT_CLASS(
    hts_plugins::HTSIKConstraintSamplerAllocator, 
    constraint_samplers::ConstraintSamplerAllocator
)