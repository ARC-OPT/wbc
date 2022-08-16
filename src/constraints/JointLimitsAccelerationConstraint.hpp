#ifndef JOINT_LIMIT_ACCELERATION_CONSTRAINT_HPP
#define JOINT_LIMIT_ACCELERATION_CONSTRAINT_HPP

#include "../core/Constraint.hpp"

#include <base/Eigen.hpp>
#include <base/Time.hpp>
#include <base/NamedVector.hpp>
#include <memory>

namespace wbc{

/**
 * @brief Abstract class to represent a generic hard constraint for a WBC optimization problem.
 */
class JointLimitsAccelerationConstraint : public Constraint {
public:

    /** @brief Default constructor */
    explicit JointLimitsAccelerationConstraint(bool reduced=false) 
        : Constraint(Constraint::bounds), reduced(reduced) { }
    
    explicit JointLimitsAccelerationConstraint(double dt, bool reduced=false);

    virtual ~JointLimitsAccelerationConstraint() = default;

    virtual void update(RobotModelPtr robot_model) override;

protected:

    /** Control timestep: used to integrate and differentiate velocities */
    double dt;

    bool reduced;

};
typedef std::shared_ptr<JointLimitsAccelerationConstraint> JointLimitsAccelerationConstraintPtr;

} // namespace wbc
#endif
