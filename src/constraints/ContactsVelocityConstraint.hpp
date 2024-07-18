#ifndef WBC_CORE_CONTACTS_VELOCITY_CONSTRAINT_HPP
#define WBC_CORE_CONTACTS_VELOCITY_CONSTRAINT_HPP

#include "../core/Constraint.hpp"
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
#endif // WBC_CORE_CONTACTS_VELOCITY_CONSTRAINT_HPP
