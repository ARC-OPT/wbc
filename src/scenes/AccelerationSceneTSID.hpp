#ifndef WBCACCELERATIONSCENE_HPP
#define WBCACCELERATIONSCENETSID_HPP

#include "../core/Scene.hpp"
#include "../core/JointAccelerationConstraint.hpp"
#include "../core/CartesianAccelerationConstraint.hpp"
#include <base/samples/Wrenches.hpp>

namespace wbc{

typedef std::shared_ptr<CartesianAccelerationConstraint> CartesianAccelerationConstraintPtr;
typedef std::shared_ptr<JointAccelerationConstraint> JointAccelerationConstraintPtr;

class AccelerationSceneTSID : public WbcScene{
protected:
    // Helper variables
    base::VectorXd q_dot;
    base::VectorXd solver_output_acc, robot_acc;
    base::samples::Wrenches contact_wrenches;
    base::MatrixXd A;
    base::VectorXd y;

    /**
     * brief Create a constraint and add it to the WBC scene
     */
    virtual ConstraintPtr createConstraint(const ConstraintConfig &config);

    base::Time stamp;

public:
    AccelerationSceneTSID(RobotModelPtr robot_model, QPSolverPtr solver) :
        WbcScene(robot_model, solver){}
    virtual ~AccelerationSceneTSID(){
    }
    /**
     * @brief Update the wbc scene and return the (updated) optimization problem
     * @param ctrl_output Control solution that fulfill the given constraints as good as possible
     */
    virtual const HierarchicalQP& update();

    /**
     * @brief Solve the given optimization problem
     * @return Solver output as joint acceleration command
     */
    virtual const base::commands::Joints& solve(const HierarchicalQP& hqp);

    /**
     * @brief evaluateConstraints Evaluate the fulfillment of the constraints given the current robot state and the solver output
     */
    virtual const ConstraintsStatus &updateConstraintsStatus(const base::samples::Joints& solver_output, const base::samples::Joints& joint_state);

    /**
     * @brief Get estimated contact wrenches
     */
    const base::samples::Wrenches& getContactWrenches(){return contact_wrenches;}
};

} // namespace wbc

#endif
