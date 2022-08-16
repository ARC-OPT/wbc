#ifndef CONTACTS_ACCELERATION_CONSTRAINT_HPP
#define CONTACTS_ACCELERATION_CONSTRAINT_HPP

#include "../core/Constraint.hpp"

#include <base/Eigen.hpp>
#include <base/Time.hpp>
#include <base/NamedVector.hpp>
#include <memory>

namespace wbc{

/**
 * @brief Abstract class to represent a generic hard constraint for a WBC optimization problem.
 */
class ContactsAccelerationConstraint : public Constraint {
public:

    /** @brief Default constructor */
    explicit ContactsAccelerationConstraint(bool _reduced = false) : Constraint(Constraint::equality), reduced(_reduced) { }

    virtual ~ContactsAccelerationConstraint() = default;

    virtual void update(RobotModelPtr robot_model) override;

private:

    bool reduced; // if torques are removed from the qp formulation or not

};

typedef std::shared_ptr<ContactsAccelerationConstraint> ContactsAccelerationConstraintPtr;

} // namespace wbc
#endif
