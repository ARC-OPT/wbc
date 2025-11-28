#ifndef WBC_CORE_RIGIDBODY_DYNAMICS_CONSTRAINT_HPP
#define WBC_CORE_RIGIDBODY_DYNAMICS_CONSTRAINT_HPP

#include "../core/Constraint.hpp"

#include <memory>

namespace wbc{

/**
 * @brief Abstract class to represent a generic hard constraint for a WBC optimization problem.
 */
class RigidbodyDynamicsConstraint : public Constraint {
public:

    /** @brief Default constructor */
    explicit RigidbodyDynamicsConstraint(bool reduced = false, uint dim_contact = 3) 
        : Constraint(Constraint::equality), reduced(reduced), dim_contact(dim_contact) { }

    virtual ~RigidbodyDynamicsConstraint() = default;

    virtual void update(RobotModelPtr robot_model) override;

protected:

    bool reduced;
    uint dim_contact;
};
typedef std::shared_ptr<RigidbodyDynamicsConstraint> RigidbodyDynamicsConstraintPtr;

} // namespace wbc
#endif // WBC_CORE_RIGIDBODY_DYNAMICS_CONSTRAINT_HPP
