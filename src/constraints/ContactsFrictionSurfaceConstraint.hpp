#ifndef WBC_CORE_CONTACTS_FRICTION_SURFACE_CONSTRAINT_HPP
#define WBC_CORE_CONTACTS_FRICTION_SURFACE_CONSTRAINT_HPP

#include "../core/Constraint.hpp"

namespace wbc {

/**
 * @brief Polyhedral wrench friction (coulomb) cone constraint according to 
 * https://scaron.info/robotics/wrench-friction-cones.html (Wrench friction cone for surface contacts)
 */
class ContactsFrictionSurfaceConstraint : public Constraint{
public:
    explicit ContactsFrictionSurfaceConstraint(bool _reduced = false, double margin = 0.0)
        : Constraint(Constraint::inequality), reduced(_reduced), margin(margin) { }

    virtual ~ContactsFrictionSurfaceConstraint() = default;

    virtual void update(RobotModelPtr robot_model) override;

private:
    bool reduced;
    double margin;  // safety margin: keeps wrench strictly inside the cone boundary
};

}

#endif // WBC_CORE_CONTACTS_FRICTION_SURFACE_CONSTRAINT_HPP
