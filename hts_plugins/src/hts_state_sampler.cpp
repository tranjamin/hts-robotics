#include <ompl/base/ValidStateSampler.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/State.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/util/RandomNumbers.h>
#include <ompl/util/Exception.h>

class HTSStateSampler : public ompl::base::ValidStateSampler {
    public:
        HTSStateSampler(const ompl::base::SpaceInformation *si) : ValidStateSampler(si) {
            name_ = "HTS state sampler";
        }

    bool sample(ompl::base::State *state) override {
        double* val = static_cast<ompl::base::RealVectorStateSpace::StateType*>(state)->values;
        double z = rng_.uniformReal(-1,1);
 
        if (z>.25 && z<.5) {
            double x = rng_.uniformReal(0,1.8), y = rng_.uniformReal(0,.2);
            switch(rng_.uniformInt(0,3)) {
                case 0: val[0]=x-1;  val[1]=y-1;  break;
                case 1: val[0]=x-.8; val[1]=y+.8; break;
                case 2: val[0]=y-1;  val[1]=x-1;  break;
                case 3: val[0]=y+.8; val[1]=x-.8; break;
            }
        } else {
            val[0] = rng_.uniformReal(-1,1);
            val[1] = rng_.uniformReal(-1,1);
        }
        
        val[2] = z;
        assert(si_->isValid(state));
        return true;
    }

    // We don't need this in the example below.
    bool sampleNear(ompl::base::State* /*state*/, const ompl::base::State* /*near*/, const double /*distance*/) override {
        throw ompl::Exception("HTSStateSampler::sampleNear", "not implemented");
        return false;
    }

protected:
    ompl::RNG rng_;
};

ompl::base::ValidStateSamplerPtr allocHTSStateSampler(const ob::SpaceInformation *si) {
    return std::make_shared<HTSStateSampler>(si);
}

class HTSDefaultStateSampler : public ompl::base::StateSampler
  {
  public:
    HTSDefaultStateSampler(const ompl::base::StateSpace* space, const moveit::core::JointModelGroup* group,
                        const moveit::core::JointBoundsVector* joint_bounds)
      : ompl::base::StateSampler(space), joint_model_group_(group), joint_bounds_(joint_bounds)
    {
    }

    void sampleUniform(ompl::base::State* state) override
    {   
      static bool print_once = false;
      if (!print_once) RCLCPP_INFO(getLogger(), "calling uniform DefaultStateSampler");
      print_once = true;

      auto* moveit_state = state->as<StateType>();

      // get a random position, so state->values now has data
      joint_model_group_->getVariableRandomPositions(moveit_rng_, moveit_state->values, *joint_bounds_);

      // update internal moveit state
      moveit_state->setJointGroupPositions(
        joint_model_group_,
        moveit_state->values
      );

      // convert it to an EE pose
      Eigen::Isometry3d& ee_tf = moveit_state->getGlobalLinkTransform("ee_link");

      // 


      state->as<StateType>()->clearKnownInformation();
    }

    void sampleUniformNear(ompl::base::State* state, const ompl::base::State* near, const double distance) override
    {   
      static bool print_once = false;
      if (!print_once) RCLCPP_INFO(getLogger(), "calling uniform near DefaultStateSampler");
      print_once = true;

      joint_model_group_->getVariableRandomPositionsNearBy(moveit_rng_, state->as<StateType>()->values, *joint_bounds_,
                                                           near->as<StateType>()->values, distance);
      state->as<StateType>()->clearKnownInformation();
    }

    void sampleGaussian(ompl::base::State* state, const ompl::base::State* mean, const double stdDev) override
    {   
      static bool print_once = false;
      if (!print_once) RCLCPP_INFO(getLogger(), "calling gaussian DefaultStateSampler");
      print_once = true;

      sampleUniformNear(state, mean, rng_.gaussian(0.0, stdDev));
    }

  protected:
    random_numbers::RandomNumberGenerator moveit_rng_;
    const moveit::core::JointModelGroup* joint_model_group_;
    const moveit::core::JointBoundsVector* joint_bounds_;
  };

class HTSConstrainedSampler : public ompl::base::ConstrainedSampler {
    public:
        HTSConstrainedSampler(const ModelBasedPlanningContext* pc, constraint_samplers::ConstraintSamplerPtr cs) : ConstrainedSampler(const ModelBasedPlanningContext* pc, constraint_samplers::ConstraintSamplerPtr cs) {
            name_ = "HTS constrained state sampler";
        }

    bool sampleUniform(ompl::base::State *state) override {
        static bool print_once = false;
        if (!print_once) RCLCPP_INFO(getLogger(), "calling uniform DefaultStateSampler");
        print_once = true;

        auto* moveit_state = state->as<StateType>();

        // get a random position, so state->values now has data
        joint_model_group_->getVariableRandomPositions(moveit_rng_, moveit_state->values, *joint_bounds_);

        // update internal moveit state
        moveit_state->setJointGroupPositions(
        joint_model_group_,
        moveit_state->values
        );

        // convert it to an EE pose
        Eigen::Isometry3d& ee_tf = moveit_state->getGlobalLinkTransform("ee_link");

        // 
    }

    // We don't need this in the example below.
    bool sampleUniformNear(ompl::base::State* state, const ompl::base::State* near, const double distance) override {
        throw ompl::Exception("HTSStateSampler::sampleNear", "not implemented");
        return false;
    }

    // We don't need this in the example below.
    bool sampleGaussian(ompl::base::State* state, const ompl::base::State* near, const double stdDev) override {
        throw ompl::Exception("HTSStateSampler::sampleGaussian", "not implemented");
        return false;
    }

protected:
    const ModelBasedPlanningContext* planning_context_;
    ompl::base::StateSamplerPtr default_;
    constraint_samplers::ConstraintSamplerPtr constraint_sampler_;
    moveit::core::RobotState work_state_;
    unsigned int constrained_success_;
    unsigned int constrained_failure_;
    double inv_dim_;
};

ompl::base::StateSamplerPtr allocHTSConstrainedStateSampler(const ob::SpaceInformation *si) {
    return std::make_shared<HTSStateSampler>(si);
}
