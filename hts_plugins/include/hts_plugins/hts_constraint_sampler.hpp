#ifndef HTS_CONSTRAINT_SAMPLER_HPP
#define HTS_CONSTRAINT_SAMPLER_HPP

#include <moveit/constraint_samplers/constraint_sampler.hpp>
#include <moveit/constraint_samplers/default_constraint_samplers.hpp>
#include <moveit/constraint_samplers/constraint_sampler_allocator.hpp>
#include <pluginlib/class_loader.hpp>

namespace hts_plugins {

    class HTSIKConstraintSampler : public constraint_samplers::IKConstraintSampler {
        public:
            HTSIKConstraintSampler(
                const std::shared_ptr<const planning_scene::PlanningScene> &scene, 
                const std::string &group_name);
    };

    class HTSIKConstraintSamplerAllocator : public constraint_samplers::ConstraintSamplerAllocator {
        public:
            HTSIKConstraintSamplerAllocator();

            std::shared_ptr<constraint_samplers::ConstraintSampler> alloc(
                const planning_scene::PlanningSceneConstPtr &scene, 
                const std::string &group_name, 
                const moveit_msgs::msg::Constraints &constr
            ) override;

            bool canService(
                const planning_scene::PlanningSceneConstPtr &scene, 
                const std::string &group_name, 
                const moveit_msgs::msg::Constraints &constr
            ) const override;
    };

} // namespace hts_plugins

#endif // HTS_CONSTRAINT_SAMPLER_HPP