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
        
        auto position = robot_model->getQ().tail(robot_model->na());
        joint_limits = robot_model->jointLimits();
        uint start_idx = nfb;

        for(size_t i = 0; i < nj-nfb; i++){
            // enforce joint velocity and position limits
            lb_vec(i+start_idx) = std::max(static_cast<double>(joint_limits.min.velocity[i]), (joint_limits.min.position[i] - position[i]) / dt);
            ub_vec(i+start_idx) = std::min(static_cast<double>(joint_limits.max.velocity[i]), (joint_limits.max.position[i] - position[i]) / dt);
            // Ensure that abs(vel) can be zero to avoid infeasibility
            lb_vec(i+start_idx) = std::min(lb_vec(i+start_idx), 0.0); 
            ub_vec(i+start_idx) = std::max(ub_vec(i+start_idx), 0.0);
        }
    }


} // namespace wbc
