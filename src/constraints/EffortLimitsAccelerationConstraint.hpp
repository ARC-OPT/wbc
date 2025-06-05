#ifndef WBC_CORE_EFFORT_LIMIT_ACCELERATION_CONSTRAINT_HPP
#define WBC_CORE_EFFORT_LIMIT_ACCELERATION_CONSTRAINT_HPP

#include "../core/Constraint.hpp"
#include "../types/JointLimits.hpp"
#include <memory>

namespace wbc{

/**
 * @brief Effort limits constraint for reduced TSID scene
 * since torques are not part of qp, torque limits are
 * enforced through acceleration and external forces limits
 * using the linear dependecy given by the dynamic model
 */
class EffortLimitsAccelerationConstraint : public Constraint {
protected:
    types::JointLimits joint_limits;
public:

    /** @brief Default constructor */
    EffortLimitsAccelerationConstraint() 
        : Constraint(Constraint::inequality) { }

    virtual ~EffortLimitsAccelerationConstraint() = default;

    virtual void update(RobotModelPtr robot_model) override;

};
typedef std::shared_ptr<EffortLimitsAccelerationConstraint> EffortLimitsAccelerationConstraintPtr;

} // namespace wbc
#endif // WBC_CORE_EFFORT_LIMIT_ACCELERATION_CONSTRAINT_HPP
