#ifndef HTS_STATE_SAMPLER_HPP
#define HTS_STATE_SAMPLER_HPP

#include <ompl/base/ValidStateSampler.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/State.h>
#include <ompl/base/Constraint.h>
#include <ompl/util/RandomNumbers.h>

#include <moveit/robot_state/robot_state.hpp>
#include <moveit/robot_model/joint_model_group.hpp>

namespace hts_plugins {

class HTSStateSampler : public ompl::base::StateSampler {
    public:
        HTSStateSampler(const ompl::base::StateSpace* space, ompl::base::ConstraintPtr constraint);
        HTSStateSampler(const ompl::base::StateSpace* space);
        void sampleUniform(ompl::base::State *state) override;
        void sampleUniformNear(ompl::base::State* state, const ompl::base::State* near, const double distance) override;
        void sampleGaussian(ompl::base::State* state, const ompl::base::State* near, const double stdDev) override;

    protected:
        ompl::RNG rng_;
        const ompl::base::StateSpace* space_;
        ompl::base::ConstraintPtr constraint_;
        
        const moveit::core::JointModelGroup* joint_model_group_;
        const moveit::core::JointBoundsVector* joint_bounds_;

};

// class HTSConstrainedSampler : public ompl::base::StateSampler {
//     public:
//         HTSConstrainedSampler(const ModelBasedPlanningContext* pc, constraint_samplers::ConstraintSamplerPtr cs);
//         bool sampleUniform(ompl::base::State *state) override;
//         bool sampleUniformNear(ompl::base::State* state, const ompl::base::State* near, const double distance) override;
//         bool sampleGaussian(ompl::base::State* state, const ompl::base::State* near, const double stdDev) override;

//     protected:
//         const ModelBasedPlanningContext* planning_context_;
//         ompl::base::StateSamplerPtr default_;
//         constraint_samplers::ConstraintSamplerPtr constraint_sampler_;
//         moveit::core::RobotState work_state_;
//         unsigned int constrained_success_;
//         unsigned int constrained_failure_;
//         double inv_dim_;
// };

// ompl::base::StateSamplerPtr allocHTSStateSampler(const ob::SpaceInformation *si);
// ompl::base::StateSamplerPtr allocHTSConstrainedStateSampler(const ob::SpaceInformation *si);

} // namespace hts_plugins

#endif // HTS_STATE_SAMPLER_HPP