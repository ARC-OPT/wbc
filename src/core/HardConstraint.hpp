#ifndef HARD_CONSTRAINT_HPP
#define HARD_CONSTRAINT_HPP

#include "RobotModel.hpp"
#include <base/Eigen.hpp>
#include <base/Time.hpp>
#include <base/NamedVector.hpp>
#include <memory>

namespace wbc{

/**
 * @brief Abstract class to represent a generic hard (linear) constraint for a WBC optimization problem.
 * the constraint belongs to one of three types:
 * equality Ax = b
 * inequality lb <= Ax <= ub
 * bounds lb <= x <= ub
 */
class HardConstraint{
public:

    enum Type {
      equality = 0,
      inequality = 1,
      bounds = 2
    };

    virtual ~HardConstraint() = default;

    /** @brief Update constraint matrix and vectors, depending on the type. Abstract method. */
    virtual void update(RobotModelPtr robot_model) = 0;

    /** @brief Return the type of this constraint */
    Type type(); 

    /** @brief return constraint matrix A */
    const base::MatrixXd& A();

    /** @brief return constraint vector b */
    const base::VectorXd& b();

    /** @brief return constraint lower bound lb */
    const base::VectorXd& lb();

    /** @brief return constraint upper bound ub */
    const base::VectorXd& ub();

    /** @brief return size of the constraint (i.e. number of rows of the constraint matrix) */
    uint size();

protected:

    /** @brief Default constructor */
    HardConstraint();

    /** @brief Constructor. Initialiye the type of this constraint */
    HardConstraint(Type type);

    Type c_type;

    /** Constraint matrix */
    base::MatrixXd A_mtx;

    /** Constraint vector */
    base::VectorXd b_vec;

    /** Constraint lower bound */
    base::VectorXd lb_vec;

    /** Constraint upper bound */
    base::VectorXd ub_vec;

};
typedef std::shared_ptr<HardConstraint> HardConstraintPtr;

} // namespace wbc
#endif
