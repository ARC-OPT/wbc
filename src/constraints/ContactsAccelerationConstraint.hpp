#ifndef WBC_CORE_CONTACTS_ACCELERATION_CONSTRAINT_HPP
#define WBC_CORE_CONTACTS_ACCELERATION_CONSTRAINT_HPP

#include "../core/Constraint.hpp"
#include <memory>

namespace wbc{

/**
 * @brief Abstract class to represent a generic hard constraint for a WBC optimization problem.
 */
class ContactsAccelerationConstraint : public Constraint {
public:

    /** @brief Default constructor */
    explicit ContactsAccelerationConstraint(bool _reduced = false, uint dim_contact = 3) : 
        Constraint(Constraint::equality), reduced(_reduced), dim_contact(dim_contact) { }

    virtual ~ContactsAccelerationConstraint() = default;

    virtual void update(RobotModelPtr robot_model) override;

private:

    bool reduced; // if torques are removed from the qp formulation or not
    uint dim_contact;

};

typedef std::shared_ptr<ContactsAccelerationConstraint> ContactsAccelerationConstraintPtr;

} // namespace wbc
#endif // WBC_CORE_CONTACTS_ACCELERATION_CONSTRAINT_HPP
