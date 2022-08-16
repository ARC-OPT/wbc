#ifndef RIGIDBODY_DYNAMICS_CONSTRAINT_HPP
#define RIGIDBODY_DYNAMICS_CONSTRAINT_HPP

#include "../core/Constraint.hpp"

#include <base/Eigen.hpp>
#include <base/Time.hpp>
#include <base/NamedVector.hpp>
#include <memory>

namespace wbc{

/**
 * @brief Abstract class to represent a generic hard constraint for a WBC optimization problem.
 */
class RigidbodyDynamicsConstraint : public Constraint {
public:

    /** @brief Default constructor */
    explicit RigidbodyDynamicsConstraint(bool reduced = false) 
        : Constraint(Constraint::equality), reduced(reduced) { }

    virtual ~RigidbodyDynamicsConstraint() = default;

    virtual void update(RobotModelPtr robot_model) override;

protected:

    bool reduced;
};
typedef std::shared_ptr<RigidbodyDynamicsConstraint> RigidbodyDynamicsConstraintPtr;

} // namespace wbc
#endif
