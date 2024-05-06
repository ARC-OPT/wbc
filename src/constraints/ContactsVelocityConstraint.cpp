#include "ContactsVelocityConstraint.hpp"

namespace wbc{

    void ContactsVelocityConstraint::update(RobotModelPtr robot_model) {
        
        const ActiveContacts& contacts = robot_model->getActiveContacts();

        uint nj = robot_model->noOfJoints();
        uint nc = contacts.size();

        A_mtx.resize(nc*3, nj);
        b_vec.resize(nc*3);
        b_vec.setZero();

        for(uint i = 0; i < nc; ++i)
            A_mtx.block(i*3, 0, 3, nj) = robot_model->bodyJacobian(robot_model->worldFrame(), contacts.names[i]).topRows<3>();
    }


} // namespace wbc
