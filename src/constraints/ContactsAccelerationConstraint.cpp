#include "ContactsAccelerationConstraint.hpp"

namespace wbc{

    void ContactsAccelerationConstraint::update(RobotModelPtr robot_model) {
        
        const ActiveContacts& contacts = robot_model->getActiveContacts();

        uint nj = robot_model->noOfJoints();
        uint na = robot_model->noOfActuatedJoints();
        uint nc = contacts.size();

        uint nv = reduced ? (nj + nc*3) : (nj + na + nc*3);

        A_mtx.resize(nc*3, nv);
        b_vec.resize(nc*3);

        A_mtx.setZero();
        b_vec.setZero();

        for(uint i = 0; i < contacts.size(); i++){
            const base::Acceleration& a = robot_model->spatialAccelerationBias(robot_model->worldFrame(), contacts.names[i]);
            b_vec.segment(i*3, 3) = -a.linear;
            A_mtx.block(i*3,  0, 3, nj) = robot_model->bodyJacobian(robot_model->worldFrame(), contacts.names[i]).topRows<3>();
        }
    }


} // namespace wbc
