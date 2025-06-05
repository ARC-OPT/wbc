#ifndef WBC_CORE_CONTACTS_FRICTION_SURFACE_CONSTRAINT_HPP
#define WBC_CORE_CONTACTS_FRICTION_SURFACE_CONSTRAINT_HPP

#include "../core/Constraint.hpp"

namespace wbc {

class ContactsFrictionSurfaceConstraint : public Constraint{
public:
    /** @brief Default constructor */
    explicit ContactsFrictionSurfaceConstraint(bool _reduced = false) : Constraint(Constraint::inequality), reduced(_reduced) { }

    virtual ~ContactsFrictionSurfaceConstraint() = default;

    virtual void update(RobotModelPtr robot_model) override;

private:
    bool reduced; // if torques are removed from the qp formulation or not
};

}

#endif // WBC_CORE_CONTACTS_FRICTION_SURFACE_CONSTRAINT_HPP
