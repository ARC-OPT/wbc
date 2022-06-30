#include "JointLimitsAccelerationHardConstraint.hpp"

namespace wbc{

    JointLimitsAccelerationHardConstraint::JointLimitsAccelerationHardConstraint(double dt) 
    :   HardConstraint(HardConstraint::bounds),
        dt(dt)
    {

    }

    void JointLimitsAccelerationHardConstraint::update(RobotModelPtr robot_model) {
        
        uint nj = robot_model->noOfJoints();
        uint na = robot_model->noOfActuatedJoints();
        uint nc = robot_model->getActiveContacts().size();

        // vars are acceleration - torques - contatc forces
        lb_vec.resize(nj+na+6*nc);
        ub_vec.resize(nj+na+6*nc);

        lb_vec.setConstant(-999999);
        ub_vec.setConstant(+999999);

        
        auto state = robot_model->jointState(robot_model->actuatedJointNames());

        // joint acceleration, velocity and position limits
        for(auto n : robot_model->actuatedJointNames()){
            size_t idx = robot_model->jointIndex(n);
            const base::JointLimitRange &range = robot_model->jointLimits().getElementByName(n);

            double pos = state[n].position;
            double vel = state[n].speed;

            // enforce joint acceleration and velocity limit
            lb_vec(idx) = std::max(static_cast<double>(range.min.acceleration), (range.min.speed - vel) / dt);
            ub_vec(idx) = std::min(static_cast<double>(range.max.acceleration), (range.max.speed - vel) / dt);
            // enforce joint position limit
            lb_vec(idx) = std::max(lb_vec(idx), 2*(range.min.position - pos - dt*vel) / (dt*dt));
            ub_vec(idx) = std::min(ub_vec(idx), 2*(range.max.position - pos - dt*vel) / (dt*dt));
        }

        // enforce joint effort limits
        for(int i = 0; i < robot_model->noOfActuatedJoints(); i++){
            const std::string& name = robot_model->actuatedJointNames()[i];
            lb_vec(i+nj) = robot_model->jointLimits()[name].min.effort;
            ub_vec(i+nj) = robot_model->jointLimits()[name].max.effort;
        }
    }


} // namespace wbc
