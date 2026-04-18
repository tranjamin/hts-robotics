#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/Constraint.h>
#include <ompl/util/Console.h>
#include <hts_plugins/hts_state_sampler.hpp>

using namespace hts_plugins;
using StateType = typename ompl::base::State;

HTSStateSampler::HTSStateSampler(const ompl::base::StateSpace* state) : ompl::base::StateSampler(state) {
    OMPL_INFORM("Calling HTSStateSampler constructor 2");
    // space_ = state;
}

HTSStateSampler::HTSStateSampler(
    const ompl::base::StateSpace* state, 
    ompl::base::ConstraintPtr constraint

) : ompl::base::StateSampler(state) {
    OMPL_INFORM("Calling HTSStateSampler constructor");
    // space_ = state;
    constraint_ = constraint;
}

void HTSStateSampler::sampleUniform(ompl::base::State *state) {
    static bool print_once = false;
    if (!print_once) OMPL_INFORM("Calling HTSStateSampler::sampleUniform");
    // print_once = true;

    // joint_model_group_->getVariableRandomPositions(rng_, state->values, *joint_bounds_);
    // state->as<StateType>()->clearKnownInformation();
}

void HTSStateSampler::sampleUniformNear(ompl::base::State *state, const ompl::base::State* near, const double distance) {
    static bool print_once = false;
    if (!print_once) OMPL_INFORM("Calling HTSStateSampler::sampleUniformNear");
    // print_once = true;

    // joint_model_group_->getVariableRandomPositionsNearBy(moveit_rng_, state->as<StateType>()->values, *joint_bounds_,
                                                        //    near->as<StateType>()->values, distance);
    // state->as<StateType>()->clearKnownInformation();

}

void HTSStateSampler::sampleGaussian(ompl::base::State *state, const ompl::base::State* near, const double stdDev) {
    static bool print_once = false;
    if (!print_once) OMPL_INFORM("Calling HTSStateSampler::sampleGaussian");
    // print_once = true;

    // sampleUniformNear(state, mean, rng_.gaussian(0.0, stdDev));
}


// ompl::base::ValidStateSamplerPtr allocHTSStateSampler(const ob::SpaceInformation *si) {
//     return std::make_shared<HTSStateSampler>(si);
// }

// used to use ompl_interface::ConstrainedSampler

// class HTSDefaultStateSampler : public ompl::base::StateSampler
//   {
//   public:
//     HTSDefaultStateSampler(const ompl::base::StateSpace* space, const moveit::core::JointModelGroup* group,
//                         const moveit::core::JointBoundsVector* joint_bounds)
//       : ompl::base::StateSampler(space), joint_model_group_(group), joint_bounds_(joint_bounds)
//     {
//     }

//     void sampleUniform(ompl::base::State* state) override
//     {   
//       static bool print_once = false;
//       if (!print_once) RCLCPP_INFO(getLogger(), "calling uniform DefaultStateSampler");
//       print_once = true;

//       auto* moveit_state = state->as<StateType>();

//       // get a random position, so state->values now has data
//       joint_model_group_->getVariableRandomPositions(moveit_rng_, moveit_state->values, *joint_bounds_);

//       // update internal moveit state
//       moveit_state->setJointGroupPositions(
//         joint_model_group_,
//         moveit_state->values
//       );

//       // convert it to an EE pose
//       Eigen::Isometry3d& ee_tf = moveit_state->getGlobalLinkTransform("ee_link");

//       // 


//       state->as<StateType>()->clearKnownInformation();
//     }

//     void sampleUniformNear(ompl::base::State* state, const ompl::base::State* near, const double distance) override
//     {   
//       static bool print_once = false;
//       if (!print_once) RCLCPP_INFO(getLogger(), "calling uniform near DefaultStateSampler");
//       print_once = true;

//       joint_model_group_->getVariableRandomPositionsNearBy(moveit_rng_, state->as<StateType>()->values, *joint_bounds_,
//                                                            near->as<StateType>()->values, distance);
//       state->as<StateType>()->clearKnownInformation();
//     }

//     void sampleGaussian(ompl::base::State* state, const ompl::base::State* mean, const double stdDev) override
//     {   
//       static bool print_once = false;
//       if (!print_once) RCLCPP_INFO(getLogger(), "calling gaussian DefaultStateSampler");
//       print_once = true;

//       sampleUniformNear(state, mean, rng_.gaussian(0.0, stdDev));
//     }

//   protected:
//     random_numbers::RandomNumberGenerator moveit_rng_;
//     const moveit::core::JointModelGroup* joint_model_group_;
//     const moveit::core::JointBoundsVector* joint_bounds_;
//   };

// class HTSConstrainedSampler : public ompl::base::ConstrainedSampler {
//     public:
//         HTSConstrainedSampler(const ModelBasedPlanningContext* pc, constraint_samplers::ConstraintSamplerPtr cs) : ConstrainedSampler(const ModelBasedPlanningContext* pc, constraint_samplers::ConstraintSamplerPtr cs) {
//             name_ = "HTS constrained state sampler";
//         }

//     bool sampleUniform(ompl::base::State *state) override {
//         static bool print_once = false;
//         if (!print_once) RCLCPP_INFO(getLogger(), "calling uniform DefaultStateSampler");
//         print_once = true;

//         auto* moveit_state = state->as<StateType>();

//         // get a random position, so state->values now has data
//         joint_model_group_->getVariableRandomPositions(moveit_rng_, moveit_state->values, *joint_bounds_);

//         // update internal moveit state
//         moveit_state->setJointGroupPositions(
//         joint_model_group_,
//         moveit_state->values
//         );

//         // convert it to an EE pose
//         Eigen::Isometry3d& ee_tf = moveit_state->getGlobalLinkTransform("ee_link");

//         // 
//         return true;
//     }

//     // We don't need this in the example below.
//     bool sampleUniformNear(ompl::base::State* state, const ompl::base::State* near, const double distance) override {
//         throw ompl::Exception("HTSStateSampler::sampleNear", "not implemented");
//         return false;
//     }

//     // We don't need this in the example below.
//     bool sampleGaussian(ompl::base::State* state, const ompl::base::State* near, const double stdDev) override {
//         throw ompl::Exception("HTSStateSampler::sampleGaussian", "not implemented");
//         return false;
//     }

// protected:
//     const ModelBasedPlanningContext* planning_context_;
//     ompl::base::StateSamplerPtr default_;
//     constraint_samplers::ConstraintSamplerPtr constraint_sampler_;
//     moveit::core::RobotState work_state_;
//     unsigned int constrained_success_;
//     unsigned int constrained_failure_;
//     double inv_dim_;
// };

// ompl::base::StateSamplerPtr allocHTSConstrainedStateSampler(const ob::SpaceInformation *si) {
//     return std::make_shared<HTSStateSampler>(si);
// }
