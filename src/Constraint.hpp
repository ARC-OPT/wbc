#ifndef CONSTRAINT_HPP
#define CONSTRAINT_HPP

#include "ConstraintConfig.hpp"
#include <base/Eigen.hpp>
#include <base/Time.hpp>
#include <base/Float.hpp>

namespace base{namespace commands{class Joints;}}
namespace base{namespace samples{class RigidBodyState;}}

namespace wbc{

/**
 * @brief Class to carry constraint specific information
 */
class Constraint{
public:

    Constraint(){}
    Constraint(const ConstraintConfig& _config) :
        config(_config),
        no_variables(_config.noOfConstraintVariables()){

        y_ref.resize(no_variables);
        y_ref_root.resize(no_variables);
        weights.resize(no_variables);
        weights_root.resize(no_variables);
        y_solution.resize(no_variables);
        y.resize(no_variables);

        _config.validate();

        reset();
    }
    virtual ~Constraint(){}

    void reset(){
        y_ref_root.setConstant(no_variables, base::NaN<double>());
        y_ref.setZero(no_variables);
        activation = config.activation;
        for(uint i = 0; i < no_variables; i++){
            weights(i) = config.weights[i];
            weights_root(i) = config.weights[i];
        }
        //Set timeout to true in the beginning. Like this, Constraints have to get a
        //reference value first to be activated, independent of the activation value
        constraint_timed_out = 1;
    }

    /** Update the Cartesian reference input for this constraint. If the Constraint is a joint space
     *  constraint, you should throw an exception*/
    virtual void setReference(const base::samples::RigidBodyState& ref){}

    /** Update the joint reference input for this constraint. If the Constraint is a Cartesian
     *  constraint, you should throw an exception*/
    virtual void setReference(const base::commands::Joints& ref){}

    /** Set constraint weights. Size of weight vector has to be same as number of constraint variables and all weights have to be >= 0 */
    void setWeights(const base::VectorXd& weights){
        if(no_variables != weights.size())
            throw std::invalid_argument("Constraint " + config.name + ": Size of weight vector should be "
                                        + std::to_string(no_variables) + " but is " + std::to_string(weights.size()));

        for(uint i = 0; i < weights.size(); i++)
            if(weights(i) < 0)
                 throw std::invalid_argument("Constraint " + config.name + ": Weights have to be >= 0, but weight "
                                             + std::to_string(i) + " is " + std::to_string(weights(i)));

        this->weights = weights;
    }

    /** Set constraint activation. Value has to be between 0 and 1. Can be used to activate(1)/deactivate(0) the constraint.*/
    void setActivation(const double activation){

        if(activation < 0 || activation > 1)
            throw std::invalid_argument("Constraint " + config.name + ": Activation has to be between 0 and 1 but is "
                                        + std::to_string(activation));
        this->activation = activation;
    }

    /** Last time an update happened on this constraint*/
    base::Time time;

    /** Configuration of this constraint. See ConstraintConfig.hpp for more details*/
    ConstraintConfig config;

    /** Reference input for this constraint. Either joint or a Cartesian space variables in ref_frame coordinates*/
    base::VectorXd y_ref;

    /** Reference value for this constraint. Either joint or a Cartesian space variables transformed to root coordinates*/
    base::VectorXd y_ref_root;

    /** Constraint weights, a 0 means that the reference of the corresponding constraint variable will be ignored while computing the solution*/
    base::VectorXd weights;

    /** Constraint weights, converted to root coordinates */
    base::VectorXd weights_root;

    /** Between 0 .. 1. Will be multiplied with the constraint weights. Can be used to (smoothly) switch on/off the constraints */
    double activation;

    /** Can be 0 or 1. Will be multiplied with the constraint weights. If no new reference values arrives for more than
     *  config.timeout time, this value will be set to zero*/
    int constraint_timed_out;

    /** Number of constraint variables */
    uint no_variables;

    /** Solution as computed by the solver for this constraint. For Cartesian constraints, this will be back transformed to
     *  Cartesian space and defined in root coordinates*/
    base::VectorXd y_solution;

    /** Actual constraint as executed on the robot. For Cartesian constraints, this will be back transformed to
     *  Cartesian space and defined in root coordinates*/
    base::VectorXd y;
};
typedef std::vector<Constraint> ConstraintsPerPrio;

} // namespace wbc
#endif
