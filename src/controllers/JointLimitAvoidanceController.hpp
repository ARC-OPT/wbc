#ifndef JOINT_LIMIT_AVOIDANCE_CONTROLLER_HPP
#define JOINT_LIMIT_AVOIDANCE_CONTROLLER_HPP

#include "PotentialField.hpp"
#include "PotentialFieldsController.hpp"
#include "../types/JointLimits.hpp"
#include "../types/JointCommand.hpp"
#include "../types/JointState.hpp"

namespace wbc {

class JointLimitAvoidanceController : public PotentialFieldsController{
protected:
    types::JointLimits joint_limits;
    types::JointCommand joints_control_output;
    double epsilon;

public:
    JointLimitAvoidanceController(const types::JointLimits& joint_limits,
                                  const std::vector<std::string> joint_names,
                                  const Eigen::VectorXd &influence_distance);
    /**
     * @brief update Compute control output
     * @return Control output. Size will be same as dimension.
     */
    const types::JointCommand& update(const types::JointState& feedback);
    /** Value to avoid nunmerical issues close to the joint limits. Default is 1e-9*/
    void setEpsilon(const double eps){epsilon = eps;}
};

}

#endif
