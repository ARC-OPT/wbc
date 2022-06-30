#ifndef JOINT_LIMIT_ACCELERATION_HARD_CONSTRAINT_HPP
#define JOINT_LIMIT_ACCELERATION_HARD_CONSTRAINT_HPP

#include "../core/HardConstraint.hpp"

#include <base/Eigen.hpp>
#include <base/Time.hpp>
#include <base/NamedVector.hpp>
#include <memory>

namespace wbc{

/**
 * @brief Abstract class to represent a generic hard constraint for a WBC optimization problem.
 */
class JointLimitsAccelerationHardConstraint : public HardConstraint {
public:

    /** @brief Default constructor */
    JointLimitsAccelerationHardConstraint() = default;
    
    JointLimitsAccelerationHardConstraint(double dt);

    virtual ~JointLimitsAccelerationHardConstraint() = default;

    virtual void update(RobotModelPtr robot_model) override;

protected:

    /** Control timestep: used to integrate and differentiate velocities */
    double dt;

};
typedef std::shared_ptr<JointLimitsAccelerationHardConstraint> JointLimitsAccelerationHardConstraintPtr;

} // namespace wbc
#endif
