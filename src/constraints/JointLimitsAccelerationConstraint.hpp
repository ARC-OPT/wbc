#ifndef WBC_CORE_JOINT_LIMIT_ACCELERATION_CONSTRAINT_HPP
#define WBC_CORE_JOINT_LIMIT_ACCELERATION_CONSTRAINT_HPP

#include "../core/Constraint.hpp"
#include <memory>

namespace wbc{

/**
 * @brief Abstract class to represent a generic hard constraint for a WBC optimization problem.
 */
class JointLimitsAccelerationConstraint : public Constraint {
public:

    /** @brief Default constructor */
    explicit JointLimitsAccelerationConstraint(bool reduced=false, uint dim_contact = 3) 
        : Constraint(Constraint::bounds), reduced(reduced), dim_contact(dim_contact) { }
    
    explicit JointLimitsAccelerationConstraint(double dt, bool reduced=false, uint dim_contact = 3);

    virtual ~JointLimitsAccelerationConstraint() = default;

    virtual void update(RobotModelPtr robot_model) override;

protected:

    /** Control timestep: used to integrate and differentiate velocities */
    double dt;
    bool reduced;
    uint dim_contact;
    types::JointLimits joint_limits;
};
typedef std::shared_ptr<JointLimitsAccelerationConstraint> JointLimitsAccelerationConstraintPtr;

} // namespace wbc
#endif // WBC_CORE_JOINT_LIMIT_ACCELERATION_CONSTRAINT_HPP
