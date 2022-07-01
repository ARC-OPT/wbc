#include "ContactsAccelerationHardConstraint.hpp"

namespace wbc{

    void ContactsAccelerationHardConstraint::update(RobotModelPtr robot_model) {
        
        const ActiveContacts& contacts = robot_model->getActiveContacts();

        uint nj = robot_model->noOfJoints();
        uint na = robot_model->noOfActuatedJoints();
        uint nc = contacts.size();

        uint nv = nj + na + nc*6;

        A_mtx.resize(nc*6, nv);
        b_vec.resize(nv);

        for(int i = 0; i < contacts.size(); i++){
            base::Acceleration a = robot_model->spatialAccelerationBias(robot_model->baseFrame(), contacts.names[i]);
            base::Vector6d acc;
            acc.segment(0,3) = a.linear;
            acc.segment(3,3) = a.angular;

            b_vec.segment(i*6, 6) = -acc;
            A_mtx.block(i*6,  0, 6, nv) = robot_model->spaceJacobian(robot_model->baseFrame(), contacts.names[i]);
        }
    }


} // namespace wbc