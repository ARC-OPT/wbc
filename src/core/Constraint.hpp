#ifndef CONSTRAINT_HPP
#define CONSTRAINT_HPP

#include "ConstraintConfig.hpp"
#include <base/Eigen.hpp>
#include <base/Time.hpp>
#include <base/NamedVector.hpp>
#include <memory>

namespace wbc{

/**
 * @brief Abstract class to represent a generic constraint for a WBC optimization problem.
 */
class Constraint{
public:

    /** @brief Default constructor */
    Constraint();

    /** @brief Resizes all members */
    Constraint(const ConstraintConfig& _config, uint n_robot_joints);
    ~Constraint();

    /**
     * @brief Reset constraint variables to initial values
     */
    void reset();

    /**
     * @brief Check if the constraint is in timeout and set the timeout flag accordingly. A constraint is in timeout if
     *    - No reference value has been set yet
     *    - A timeout value is configured (config.timeout > 0) and no reference value arrived during the timeout period
     */
    void checkTimeout();

    /**
     * @brief Set constraint weights.
     * @param weights Weight vector. Size has to be same as number of constraint variables and all entries have to be >= 0
     */
    void setWeights(const base::VectorXd& weights);

    /**
     * @brief Set constraint activation.
     * @param activation Value has to be between 0 and 1. Can be used to activate(1)/deactivate(0) the constraint.
     */
    void setActivation(const double activation);

    /** Last time the constraint reference values was updated.*/
    base::Time time;

    /** Configuration of this constraint. See ConstraintConfig.hpp for more details*/
    ConstraintConfig config;

    /** Reference input for this constraint. Can be either joint or a Cartesian space variables. If the latter is the case
     *  they have to be expressed in in ref_frame coordinates! See ConstraintConfig.hpp for more details.*/
    base::VectorXd y_ref;

    /** Reference value for this constraint. Can be either joint or a Cartesian space variables. In the former case, y_ref_root will be
     *  equal to y_ref, in the latter case, y_ref_root will be y_ref transformed into the robot root/base frame.*/
    base::VectorXd y_ref_root;

    /** Constraint weights. Size has to be same as number of constraint variables and all entries have to be >= 0.
     *  A zero entry means that the reference of the corresponding constraint variable will be ignored while computing the solution, for example
     *  when controlling the Cartesian pose, the last 3 entries can be set to zero in order to ignore the orientarion and only control the position*/
    base::VectorXd weights;

    /** Constraint weights. In case of joint constraints, weights_root will be equal to weights. In case of Cartesian constraints, weights_root will be equal to
      * weights, transformed into the robot's base/root frame*/
    base::VectorXd weights_root;

    /** Constraint activation. Has to be between 0 and 1. Will be multiplied with the constraint weights. Can be used to (smoothly) switch on/off the constraints */
    double activation;

    /** Can be 0 or 1. Will be multiplied with the constraint weights. If no new reference values arrives for more than
     *  config.timeout time, this value will be set to zero*/
    int timeout;

    /** Constraint matrix */
    base::MatrixXd A;
};
typedef std::shared_ptr<Constraint> ConstraintPtr;

} // namespace wbc
#endif
