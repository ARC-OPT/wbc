#ifndef CONSTRAINT_HPP
#define CONSTRAINT_HPP

#include <base/Eigen.hpp>
#include "ConstraintConfig.hpp"
#include <base/time.h>
#include <base/samples/RigidBodyState.hpp>
#include <base/samples/Joints.hpp>
#include <base/logging.h>

namespace wbc{

/**
 * @brief helper class to carry sub constraint specific information
 */
class Constraint{
public:
    Constraint(){}
    Constraint(const ConstraintConfig& _config,
            const uint _no_robot_joints,
            bool _constraints_active  = true){

        constraints_initially_active = _constraints_active;
        config = _config;
        if(config.type == jnt)
            no_variables = _config.joint_names.size();
        else
            no_variables = 6;
        y_des.resize(no_variables);
        y_solution.resize(no_variables);
        y.resize(no_variables);
        y_des_root_frame.resize(no_variables);
        weights = base::VectorXd::Ones(no_variables);
        A.resize(no_variables, _no_robot_joints);

        reset();
    }

    ConstraintConfig config;
    base::Time time;

    base::VectorXd y_des; /** Reference value for Constraint */
    base::VectorXd y_des_root_frame; /** Reference value for Constraint, transformed to root frame of Constraint */
    base::VectorXd y_solution; /** Solution for this Constraint from wbc */
    base::VectorXd y; /** Actual value of a constraint (on the real robot)*/
    base::VectorXd weights; /** constraint weights, a 0 means that the reference of the corresponding constraint variable will be ignored while computing the solution*/
    double activation; /** Between 0 .. 1. Will be multiplied with the constraint weights. Can be used to (smoothly) switch on/off the constraints */
    int constraint_timed_out; /** May be 0 or 1. Will be multiplied with the constraint weights. If no new reference values arrive for a certain time,the constraint times out*/
    double manipulability; /** Manipulability of the kinematic chain connected with this sub constraint*/

    base::MatrixXd A; /** constraint matrix */
    bool constraints_initially_active;
    uint no_variables; /** Number of constraint variables */
    base::Time last_ref_input; /** last time a new reference sample arrived*/

    void setReference(const base::samples::RigidBodyState &ref);
    void setReference(const base::samples::Joints& ref);
    void updateTime();
    void validate();
    void computeDebug(const base::VectorXd& solver_output, const base::VectorXd& robot_vel);
    void reset();
};

}
#endif
