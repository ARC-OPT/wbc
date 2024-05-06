#include "EffortLimitsAccelerationConstraint.hpp"

namespace wbc{

    void EffortLimitsAccelerationConstraint::update(RobotModelPtr robot_model) {
        
        const auto& contacts = robot_model->getActiveContacts();

        uint nj = robot_model->noOfJoints();
        uint na = robot_model->noOfActuatedJoints();
        uint nc = contacts.size();

        lb_vec.resize(na);
        ub_vec.resize(na);

        lb_vec.setConstant(-10000);
        ub_vec.setConstant(+10000);
        
        A_mtx.resize(na, nj+6*nc);
        A_mtx.setZero();

        //! NOTE! -> not considering selection matrix

        A_mtx.block(0,0,na,nj) = robot_model->jointSpaceInertiaMatrix().bottomRows(na);
        for(uint i=0; i < nc; ++i){
            if(contacts[i].active){
                A_mtx.block(0,nj+i*6,na,6) = -robot_model->bodyJacobian(robot_model->worldFrame(), contacts.names[i]).transpose().bottomRows(na);
            }
        }

        // enforce joint effort limits (only if torques are part of the optimization problem)
        Eigen::VectorXd b = robot_model->biasForces().tail(na);

        for(uint i = 0; i < robot_model->noOfActuatedJoints(); i++){
            const std::string& name = robot_model->actuatedJointNames()[i];
            lb_vec(i) = robot_model->jointLimits()[name].min.effort - b(i);
            ub_vec(i) = robot_model->jointLimits()[name].max.effort - b(i);
        }
    }


} // namespace wbc
