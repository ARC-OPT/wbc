#include "JointLimitsAccelerationConstraint.hpp"
#include <iostream>
namespace wbc{

    JointLimitsAccelerationConstraint::JointLimitsAccelerationConstraint(double dt, bool reduced, uint dim_contact) 
    :   Constraint(Constraint::bounds),
        dt(dt),
        reduced(reduced),
        dim_contact(dim_contact)
    {

    }

    void JointLimitsAccelerationConstraint::update(RobotModelPtr robot_model) {
        
        uint nj = robot_model->nj();
        uint na = robot_model->na();
        uint nc = robot_model->nc();
        uint nfb = robot_model->nfb();
        uint nv = reduced ? nj+dim_contact*nc : nj+na+dim_contact*nc;

        lb_vec.resize(nv);
        ub_vec.resize(nv);
        lb_vec.setConstant(-10000);
        ub_vec.setConstant(+10000);

        bool check_accelerations = true;
        bool check_velocities = false;
        bool check_positions = false;
        
        auto position = robot_model->getQ().tail(robot_model->na());
        auto velocity = robot_model->getQd().tail(robot_model->na());

        // check if a value is not nan otherwise return a substiture value
        // used for acceleration limits since they might not be define in URDFs
        auto check_number = [](double val, double sub){
            if(std::isnan(val))
                return sub;
            return val;
        };

        // joint acceleration, velocity and position limits
        joint_limits = robot_model->jointLimits();
        uint start_idx = nfb;

        for(size_t i = 0; i < nj - nfb; i++){

            double pos = position[i];
            double vel = velocity[i];

            // enforce joint acceleration and velocity limit
            if(check_accelerations)
            {
                lb_vec(i+start_idx) = check_number(joint_limits.min.acceleration[i], -10000);
                ub_vec(i+start_idx) = check_number(joint_limits.max.acceleration[i], +10000);
            }
            if(check_velocities)
            {
                lb_vec(i+start_idx) = std::max(lb_vec(i+start_idx), (joint_limits.min.velocity[i] - vel) / dt);
                ub_vec(i+start_idx) = std::min(ub_vec(i+start_idx), (joint_limits.max.velocity[i] - vel) / dt);
            }
            // enforce joint position limit
            if(check_positions)
            {
                lb_vec(i+start_idx) = std::max(lb_vec(i+start_idx), 2*(joint_limits.min.position[i] - pos - dt*vel) / (dt*dt));
                ub_vec(i+start_idx) = std::min(ub_vec(i+start_idx), 2*(joint_limits.max.position[i] - pos - dt*vel) / (dt*dt));
            }
        }

        // enforce joint effort limits (only if torques are part of the optimization problem)
        // otherwise use EffortLimitsAccelerationConstraint
        if(reduced)
            return;

        for(size_t i = 0; i < na; i++){
            lb_vec(i+nj) = joint_limits.min.effort[i];
            ub_vec(i+nj) = joint_limits.max.effort[i];
        }
    }


} // namespace wbc
