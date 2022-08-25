#include "JointLimitsVelocityConstraint.hpp"

namespace wbc{

    JointLimitsVelocityConstraint::JointLimitsVelocityConstraint(double dt) :
        Constraint(Constraint::bounds), 
        dt(dt)
    {

    }

    void JointLimitsVelocityConstraint::update(RobotModelPtr robot_model) {

        uint nj = robot_model->noOfJoints();

        // vars are velocities
        lb_vec.resize(nj);
        ub_vec.resize(nj);

        lb_vec.setConstant(-999999);
        ub_vec.setConstant(+999999);
        
        auto state = robot_model->jointState(robot_model->actuatedJointNames());

        for(auto n : robot_model->actuatedJointNames()){
            size_t idx = robot_model->jointIndex(n);
            const base::JointLimitRange &range = robot_model->jointLimits().getElementByName(n);

            // enforce joint velocity and position limits
            lb_vec(idx) = std::max(static_cast<double>(range.min.speed), (range.min.position - state[n].position) / dt);
            ub_vec(idx) = std::min(static_cast<double>(range.max.speed), (range.max.position - state[n].position) / dt);
            lb_vec(idx) = std::min(lb_vec(idx), 0.0); // Why is this required?
            ub_vec(idx) = std::max(ub_vec(idx), 0.0);
        }
    }


} // namespace wbc
