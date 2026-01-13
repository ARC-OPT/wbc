#include "EffortLimitsAccelerationConstraint.hpp"

namespace wbc{

    void EffortLimitsAccelerationConstraint::update(RobotModelPtr robot_model) {
        
        const auto& contacts = robot_model->getContacts();

        uint nj = robot_model->nj();
        uint na = robot_model->na();
        uint nc = contacts.size();

        lb_vec.resize(na);
        ub_vec.resize(na);

        lb_vec.setConstant(-10000);
        ub_vec.setConstant(+10000);
        
        A_mtx.resize(na, nj+dim_contact*nc);
        A_mtx.setZero();

        //! NOTE! -> not considering selection matrix

        A_mtx.block(0,0,na,nj) = robot_model->jointSpaceInertiaMatrix().bottomRows(na);
        for(uint i=0; i < nc; ++i){
            if(contacts[i].active){
                A_mtx.block(0,nj+i*dim_contact,na,dim_contact) = -robot_model->spaceJacobian(contacts[i].frame_id).topRows(dim_contact).transpose().bottomRows(na);
            }
        }

        // enforce joint effort limits (only if torques are part of the optimization problem)
        Eigen::VectorXd b = robot_model->biasForces().tail(na);

        joint_limits = robot_model->jointLimits();

        for(uint i = 0; i < na; i++){
            assert(joint_limits.min.effort[i]  joint_limits.max.effort[i] && "EffortLimitsAccelerationConstraint: joint effort limits are inconsistent!");
            lb_vec(i) = joint_limits.min.effort[i] - b(i);
            ub_vec(i) = joint_limits.max.effort[i] - b(i);
        }
    }


} // namespace wbc
