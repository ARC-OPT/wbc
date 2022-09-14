#ifndef CONTACTS_VELOCITY_CONSTRAINT_HPP
#define CONTACTS_VELOCITY_CONSTRAINT_HPP

#include "../core/Constraint.hpp"

#include <base/Eigen.hpp>
#include <base/Time.hpp>
#include <base/NamedVector.hpp>
#include <memory>

namespace wbc{

/**
 * @brief Abstract class to represent a generic hard constraint for a WBC optimization problem.
 */
class ContactsVelocityConstraint : public Constraint {
public:

    /** @brief Default constructor */
    ContactsVelocityConstraint() : Constraint(Constraint::equality) { }

    virtual ~ContactsVelocityConstraint() = default;

    virtual void update(RobotModelPtr robot_model) override;

};
typedef std::shared_ptr<ContactsVelocityConstraint> ContactsVelocityConstraintPtr;

} // namespace wbc
#endif
