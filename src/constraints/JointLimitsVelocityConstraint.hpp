#ifndef WBC_CORE_JOINT_LIMIT_VELOCITY_CONSTRAINT_HPP
#define WBC_CORE_JOINT_LIMIT_VELOCITY_CONSTRAINT_HPP

#include "../core/Constraint.hpp"
#include "../types/JointLimits.hpp"
#include <memory>

namespace wbc{

/**
 * @brief Abstract class to represent a generic hard constraint for a WBC optimization problem.
 */
class JointLimitsVelocityConstraint : public Constraint {
public:

    /** @brief Default constructor */
    JointLimitsVelocityConstraint() : Constraint(Constraint::bounds) { }

    JointLimitsVelocityConstraint(double dt);

    virtual ~JointLimitsVelocityConstraint() = default;

    virtual void update(RobotModelPtr robot_model) override;

protected:

    /** Control timestep: used to integrate and differentiate velocities */
    double dt;
    types::JointLimits joint_limits;

};
typedef std::shared_ptr<JointLimitsVelocityConstraint> JointLimitsVelocityConstraintPtr;

} // namespace wbc
#endif // WBC_CORE_JOINT_LIMIT_VELOCITY_CONSTRAINT_HPP
