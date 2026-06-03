#ifndef WBC_CORE_CONTACTS_FRICTION_POINT_CONSTRAINT_HPP
#define WBC_CORE_CONTACTS_FRICTION_POINT_CONSTRAINT_HPP

#include "../core/Constraint.hpp"

namespace wbc {

class ContactsFrictionPointConstraint : public Constraint{
public:
    explicit ContactsFrictionPointConstraint(bool _reduced = false, double margin = 0.0)
        : Constraint(Constraint::inequality), reduced(_reduced), margin(margin) { }

    virtual ~ContactsFrictionPointConstraint() = default;

    virtual void update(RobotModelPtr robot_model) override;

private:
    bool reduced;
    double margin;
};

}

#endif // WBC_CORE_CONTACTS_FRICTION_POINT_CONSTRAINT_HPP
