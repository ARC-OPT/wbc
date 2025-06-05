#include "JointLimitsVelocityConstraint.hpp"

namespace wbc{

    JointLimitsVelocityConstraint::JointLimitsVelocityConstraint(double dt) :
        Constraint(Constraint::bounds), 
        dt(dt)
    {

    }

    void JointLimitsVelocityConstraint::update(RobotModelPtr robot_model) {

        uint nj = robot_model->nj();
        uint nfb = robot_model->nfb();

        // vars are velocities
        lb_vec.resize(nj);
        ub_vec.resize(nj);

        lb_vec.setConstant(-999999);
        ub_vec.setConstant(+999999);
        
        auto state = robot_model->jointState();
        joint_limits = robot_model->jointLimits();
        uint start_idx = nfb;

        for(size_t i = 0; i < nj-nfb; i++){
            // enforce joint velocity and position limits
            lb_vec(i+start_idx) = std::max(static_cast<double>(joint_limits.min.velocity[i]), (joint_limits.min.position[i] - state.position[i]) / dt);
            ub_vec(i+start_idx) = std::min(static_cast<double>(joint_limits.max.velocity[i]), (joint_limits.max.position[i] - state.position[i]) / dt);
            lb_vec(i+start_idx) = std::min(lb_vec(i+start_idx), 0.0); // Why is this required?
            ub_vec(i+start_idx) = std::max(ub_vec(i+start_idx), 0.0);
        }
    }


} // namespace wbc
