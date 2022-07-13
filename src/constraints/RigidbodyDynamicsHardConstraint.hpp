#ifndef RIGIDBODY_DYNAMICS_HARD_CONSTRAINT_HPP
#define RIGIDBODY_DYNAMICS_HARD_CONSTRAINT_HPP

#include "../core/HardConstraint.hpp"

#include <base/Eigen.hpp>
#include <base/Time.hpp>
#include <base/NamedVector.hpp>
#include <memory>

namespace wbc{

/**
 * @brief Abstract class to represent a generic hard constraint for a WBC optimization problem.
 */
class RigidbodyDynamicsHardConstraint : public HardConstraint {
public:

    /** @brief Default constructor */
    RigidbodyDynamicsHardConstraint() = default;

    virtual ~RigidbodyDynamicsHardConstraint() = default;

    virtual void update(RobotModelPtr robot_model) override;

};
typedef std::shared_ptr<RigidbodyDynamicsHardConstraint> RigidbodyDynamicsHardConstraintPtr;

} // namespace wbc
#endif
