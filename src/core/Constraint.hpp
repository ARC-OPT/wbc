#ifndef WBC_CORE_CONSTRAINT_HPP
#define WBC_CORE_CONSTRAINT_HPP

#include "RobotModel.hpp"
#include <Eigen/Core>
#include <memory>

namespace wbc{

/**
 * @brief Abstract class to represent a generic hard (linear) constraint for a WBC optimization problem.
 * the constraint belongs to one of three types:
 * equality Ax = b
 * inequality lb <= Ax <= ub
 * bounds lb <= x <= ub
 */
class Constraint{
public:

    enum Type {
      equality = 0,
      inequality = 1,
      bounds = 2
    };

    virtual ~Constraint() = default;

    /** @brief Update constraint matrix and vectors, depending on the type. Abstract method. */
    virtual void update(RobotModelPtr robot_model) = 0;

    /** @brief Return the type of this constraint */
    Type type(); 

    /** @brief return constraint matrix A */
    const Eigen::MatrixXd& A();

    /** @brief return constraint vector b */
    const Eigen::VectorXd& b();

    /** @brief return constraint lower bound lb */
    const Eigen::VectorXd& lb();

    /** @brief return constraint upper bound ub */
    const Eigen::VectorXd& ub();

    /** @brief return size of the constraint (i.e. number of rows of the constraint matrix) */
    uint size();

protected:

    /** @brief Default constructor */
    Constraint();

    /** @brief Constructor. Initialiye the type of this constraint */
    Constraint(Type type);

    Type c_type;

    /** Constraint matrix */
    Eigen::MatrixXd A_mtx;

    /** Constraint vector */
    Eigen::VectorXd b_vec;

    /** Constraint lower bound */
    Eigen::VectorXd lb_vec;

    /** Constraint upper bound */
    Eigen::VectorXd ub_vec;

};
typedef std::shared_ptr<Constraint> ConstraintPtr;

} // namespace wbc
#endif // WBC_CORE_CONSTRAINT_HPP
