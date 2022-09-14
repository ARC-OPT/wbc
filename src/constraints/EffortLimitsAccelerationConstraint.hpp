#ifndef EFFORT_LIMIT_ACCELERATION_CONSTRAINT_HPP
#define EFFORT_LIMIT_ACCELERATION_CONSTRAINT_HPP

#include "../core/Constraint.hpp"

#include <base/Eigen.hpp>
#include <base/Time.hpp>
#include <base/NamedVector.hpp>
#include <memory>

namespace wbc{

/**
 * @brief Effort limits constraint for reduced TSID scene
 * since torques are not part of qp, torque limits are
 * enforced through acceleration and external forces limits
 * using the linear dependecy given by the dynamic model
 */
class EffortLimitsAccelerationConstraint : public Constraint {
public:

    /** @brief Default constructor */
    EffortLimitsAccelerationConstraint() 
        : Constraint(Constraint::inequality) { }

    virtual ~EffortLimitsAccelerationConstraint() = default;

    virtual void update(RobotModelPtr robot_model) override;

};
typedef std::shared_ptr<EffortLimitsAccelerationConstraint> EffortLimitsAccelerationConstraintPtr;

} // namespace wbc
#endif
