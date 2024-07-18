#ifndef WBC_CORE_CONTACTS_FRICTION_POINT_CONSTRAINT_HPP
#define WBC_CORE_CONTACTS_FRICTION_POINT_CONSTRAINT_HPP

#include "../core/Constraint.hpp"

namespace wbc {

class ContactsFrictionPointConstraint : public Constraint{
public:
    /** @brief Default constructor */
    explicit ContactsFrictionPointConstraint(bool _reduced = false) : Constraint(Constraint::inequality), reduced(_reduced) { }

    virtual ~ContactsFrictionPointConstraint() = default;

    virtual void update(RobotModelPtr robot_model) override;

private:
    bool reduced; // if torques are removed from the qp formulation or not
};

}

#endif // WBC_CORE_CONTACTS_FRICTION_POINT_CONSTRAINT_HPP
