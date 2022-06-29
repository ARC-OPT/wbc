#ifndef CONSTRAINT_HPP
#define CONSTRAINT_HPP

#include "ConstraintConfig.hpp"
#include <base/Eigen.hpp>
#include <base/Time.hpp>
#include <base/NamedVector.hpp>
#include <memory>

namespace wbc{

/**
 * @brief Abstract class to represent a generic hard constraint for a WBC optimization problem.
 */
class HardConstraint{
public:

    enum Type {
      equality = 0,
      inequality = 1,
      bounds = 2
    };

    ~HardConstraint();

    virtual void update() = 0;

    const Type type(); 

    const base::MatrixXd& A();

    const base::VectorXd& b();

    const base::VectorXd& lb();

    const base::VectorXd& ub();

protected:

    /** @brief Default constructor */
    HardConstraint();

    /** @brief Resizes all members */
    HardConstraint(Type type, uint n_robot_joints);

    Type type;

    /** Number of joints in the constraint*/
    uint number_of_joints;

    /** Constraint matrix */
    base::MatrixXd A;

    /** Constraint lower bound */
    base::VectorXd lb;

    /** Constraint upper bound */
    base::VectorXd ub;

    /** Joint mask vector. Values among {0,1}. 1 will activate  */
    base::VectorXi mask;

};
typedef std::shared_ptr<HardConstraint> HardConstraintPtr;

} // namespace wbc
#endif
