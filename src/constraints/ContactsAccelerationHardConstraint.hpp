#ifndef CONTACTS_ACCELERATION_HARD_CONSTRAINT_HPP
#define CONTACTS_ACCELERATION_HARD_CONSTRAINT_HPP

#include "../core/HardConstraint.hpp"

#include <base/Eigen.hpp>
#include <base/Time.hpp>
#include <base/NamedVector.hpp>
#include <memory>

namespace wbc{

/**
 * @brief Abstract class to represent a generic hard constraint for a WBC optimization problem.
 */
class ContactsAccelerationHardConstraint : public HardConstraint {
public:

    /** @brief Default constructor */
    ContactsAccelerationHardConstraint() = default;

    virtual ~ContactsAccelerationHardConstraint() = default;

    virtual void update(RobotModelPtr robot_model) override;

};
typedef std::shared_ptr<ContactsAccelerationHardConstraint> ContactsAccelerationHardConstraintPtr;

} // namespace wbc
#endif
