#include "JointLimitsVelocityConstraintConfig.hpp"
#include <base/Eigen.hpp>
#include <base/Time.hpp>
#include <base/NamedVector.hpp>
#include <memory>

namespace wbc{

    /** @brief Default constructor */
    JointLimitsVelocityHardConstraint::JointLimitsVelocityHardConstraint() {

    }


    JointLimitsVelocityHardConstraint::JointLimitsVelocityHardConstraint(uint n_robot_joints, double dt) 
    :   HardConstraint(HardConstraint::bounds, n_robot_joints + ){

    }

    JointLimitsVelocityHardConstraint::~JointLimitsVelocityHardConstraint() { 

    }

    void JointLimitsVelocityHardConstraint::update(RobotModelPtr robot_model) {
        
        for(auto n : robot_model->actuatedJointNames()){
            size_t idx = robot_model->jointIndex(n);
            const base::JointLimitRange &range = robot_model->jointLimits().getElementByName(n);

            //
            lb(idx) = std::max(range.min.speed, (range.min.pos - robot_model->position(idx)) / dt);
            ub(idx) = std::min(range.max.speed, (range.max.pos - robot_model->position(idx)) / dt);
        }
    }


} // namespace wbc
#endif
