#include "ContactsVelocityConstraint.hpp"

namespace wbc{

    void ContactsVelocityConstraint::update(RobotModelPtr robot_model) {
        
        const ActiveContacts& contacts = robot_model->getActiveContacts();

        uint nj = robot_model->noOfJoints();
        uint nc = contacts.size();

        A_mtx.resize(nc*6, nj);
        b_vec.resize(nc*6);
        b_vec.setZero();

        for(int i = 0; i < nc; ++i)
            A_mtx.block(i*6, 0, 6, nj) = contacts[i].active * robot_model->bodyJacobian(robot_model->worldFrame(), contacts.names[i]);
    }


} // namespace wbc
