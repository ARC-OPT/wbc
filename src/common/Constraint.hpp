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
 * @brief Class to carry constraint specific information
 */
class Constraint{
public:

    Constraint(){
    }
    Constraint(const ConstraintConfig& _config)
    {

        config = _config;

        if(config.type == jnt)
            no_variables = _config.joint_names.size();
        else
            no_variables = 6;

        if(config.weights.size() != no_variables){
            LOG_ERROR("Constraint '%s' has %i variables, but its weights vector has size %i",
                      config.name.c_str(), no_variables, config.weights.size());
            throw std::invalid_argument("Invalid WBC config");
        }
        if(config.activation < 0 || config.activation > 1){
            LOG_ERROR("Activation of constraint '%s' is %f. It has to be be between 0 and 1",
                      config.name.c_str(),config.activation);
            throw std::invalid_argument("Invalid WBC config");
        }

        for(uint i = 0; i < config.weights.size(); i++)
        {
            if(config.weights[i] < 0){
                LOG_ERROR("Weight no %i of constraint '%s' is %f. It has to be >= 0",
                          i, config.name.c_str(), config.weights[i]);
                throw std::invalid_argument("Invalid WBC config");

            }
        }

        y_ref.resize(no_variables);
        y_ref_root.resize(no_variables);
        weights.resize(no_variables);
        weights_root.resize(no_variables);
        y_solution.resize(no_variables);
        y.resize(no_variables);

        reset();
    }

    base::Time time;

    /** Configuration of this constraint. */
    ConstraintConfig config;

    /** Reference value for this constraint. Either joint space values or a Cartesian Twist defined in ref_frame coordinates*/
    base::VectorXd y_ref;

    /** Reference value for this constraint. Either joint space values or a Cartesian Twist defined in root coordinates*/
    base::VectorXd y_ref_root;

    /** Constraint weights, a 0 means that the reference of the corresponding constraint variable will be ignored while computing the solution*/
    base::VectorXd weights;

    /** Constraint weights, converted to root coordinates */
    base::VectorXd weights_root;

    /** Between 0 .. 1. Will be multiplied with the constraint weights. Can be used to (smoothly) switch on/off the constraints */
    double activation;

    /** Can be 0 or 1. Will be multiplied with the constraint weights. If no new reference values arrives for a certain time, the constraint times out*/
    int constraint_timed_out;

    /** Number of constraint variables */
    uint no_variables;

    /** last time a new reference sample arrived*/
    base::Time last_ref_input;

    /** Solution as computed by the solver for this constraint. For Cartesian constraints, this will be back transformed to
     *  Cartesian space and defined in root coordinates*/
    base::VectorXd y_solution;

    /** Actual constraint as executed on the robot. For Cartesian constraints, this will be back transformed to
     *  Cartesian space and defined in root coordinates*/
    base::VectorXd y;

    void setReference(const base::samples::RigidBodyState &ref);
    void setReference(const base::samples::Joints& ref);
    void validate();
    void reset();
};

}
#endif
