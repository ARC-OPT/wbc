#ifndef CONTACTS_FRICTION_CONSTRAINT_HPP
#define CONTACTS_FRICTION_CONSTRAINT_HPP

#include "../core/Constraint.hpp"

namespace wbc {

class ContactsFrictionConstraint : public Constraint{
public:
    /** @brief Default constructor */
    explicit ContactsFrictionConstraint(bool _reduced = false) : Constraint(Constraint::inequality), reduced(_reduced) { }

    virtual ~ContactsFrictionConstraint() = default;

    virtual void update(RobotModelPtr robot_model) override;

private:
    bool reduced; // if torques are removed from the qp formulation or not
};

}

#endif
