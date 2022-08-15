#include "JointLimitsAccelerationConstraint.hpp"

namespace wbc{

    JointLimitsAccelerationConstraint::JointLimitsAccelerationConstraint(double dt) 
    :   Constraint(Constraint::bounds),
        dt(dt)
    {

    }

    void JointLimitsAccelerationConstraint::update(RobotModelPtr robot_model) {
        
        uint nj = robot_model->noOfJoints();
        uint na = robot_model->noOfActuatedJoints();
        uint nc = robot_model->getActiveContacts().size();

        // vars are acceleration - torques - contatc forces
        lb_vec.resize(nj+na+6*nc);
        ub_vec.resize(nj+na+6*nc);

        lb_vec.setConstant(-999999);
        ub_vec.setConstant(+999999);

        
        auto state = robot_model->jointState(robot_model->actuatedJointNames());

        // check if a value is not nan otherwise return a substiture value
        // used for acceleration limits since they might not be define in URDFs
        auto check_number = [](double val, double sub){
            if(std::isnan(val))
                return sub;
            return val;
        };

        // joint acceleration, velocity and position limits
        for(auto n : robot_model->actuatedJointNames()){
            size_t idx = robot_model->jointIndex(n);
            const base::JointLimitRange &range = robot_model->jointLimits().getElementByName(n);

            double pos = state[n].position;
            double vel = state[n].speed;

            // enforce joint acceleration and velocity limit
            lb_vec(idx) = std::max(check_number(range.min.acceleration, -999999), (range.min.speed - vel) / dt);
            ub_vec(idx) = std::min(check_number(range.max.acceleration, +999999), (range.max.speed - vel) / dt);
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
