#ifndef CONSTRAINT_HPP
#define CONSTRAINT_HPP

#include "ConstraintConfig.hpp"
#include <base/Eigen.hpp>
#include <base/Time.hpp>
#include <base/NamedVector.hpp>
#include <memory>

namespace wbc{

/**
 * @brief Abstract class to represent a generic hard constraint for a WBC optimization problem.
 */
class JointLimitsVelocityHardConstraint : public HardConstraint {
public:

    /** @brief Default constructor */
    JointLimitsVelocityHardConstraint();


    JointLimitsVelocityHardConstraint(uint n_robot_joints, double dt);

    ~JointLimitsVelocityHardConstraint();

    virtual void update(RobotModelPtr robot_model) = 0;

protected:

    /** Control timestep: used to integrate and differentiate velocities */
    double dt;

};
typedef std::shared_ptr<JointLimitsVelocityHardConstraint> JointLimitsVelocityHardConstraintPtr;

} // namespace wbc
#endif
