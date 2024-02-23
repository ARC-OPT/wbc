#ifndef JOINT_LIMIT_AVOIDANCE_CONTROLLER_HPP
#define JOINT_LIMIT_AVOIDANCE_CONTROLLER_HPP

#include "PotentialField.hpp"
#include "PotentialFieldsController.hpp"
#include <base/JointLimits.hpp>
#include <base/commands/Joints.hpp>

namespace wbc {

class JointLimitAvoidanceController : public PotentialFieldsController{
protected:
    base::JointLimits joint_limits;
    base::commands::Joints joints_control_output;
    double epsilon;

public:
    JointLimitAvoidanceController(const base::JointLimits& limits,
                                  const base::VectorXd &influence_distance);
    /**
     * @brief update Compute control output
     * @return Control output. Size will be same as dimension.
     */
    const base::commands::Joints& update(const base::samples::Joints& feedback);
    /**
     * @brief setJointLimits set upper and lower joint limits for the controller
     * @param joint_limits
     */
    void setJointLimits(const base::JointLimits& limits);
    /** Value to avoid nunmerical issues close to the joint limits. Default is 1e-9*/
    void setEpsilon(const double eps){epsilon = eps;}
};

}

#endif
