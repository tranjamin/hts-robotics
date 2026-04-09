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


class HTSConstrainedSampler : public ompl::base::ConstrainedSampler {
    public:
        HTSConstrainedSampler(const ModelBasedPlanningContext* pc, constraint_samplers::ConstraintSamplerPtr cs) : ConstrainedSampler(const ModelBasedPlanningContext* pc, constraint_samplers::ConstraintSamplerPtr cs) {
            name_ = "HTS constrained state sampler";
        }

    bool sampleUniform(ompl::base::State *state) override {

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
    ompl::RNG rng_;
};

ompl::base::StateSamplerPtr allocHTSConstrainedStateSampler(const ob::SpaceInformation *si) {
    return std::make_shared<HTSStateSampler>(si);
}
