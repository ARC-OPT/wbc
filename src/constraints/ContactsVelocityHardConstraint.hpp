#ifndef CONTACTS_VELOCITY_HARD_CONSTRAINT_HPP
#define CONTACTS_VELOCITY_HARD_CONSTRAINT_HPP

#include "../core/HardConstraint.hpp"

#include <base/Eigen.hpp>
#include <base/Time.hpp>
#include <base/NamedVector.hpp>
#include <memory>

namespace wbc{

/**
 * @brief Abstract class to represent a generic hard constraint for a WBC optimization problem.
 */
class ContactsVelocityHardConstraint : public HardConstraint {
public:

    /** @brief Default constructor */
    ContactsVelocityHardConstraint() = default;

    ~ContactsVelocityHardConstraint() = default;

    virtual void update(RobotModelPtr robot_model) override;

};
typedef std::shared_ptr<ContactsVelocityHardConstraint> ContactsVelocityHardConstraintPtr;

} // namespace wbc
#endif
