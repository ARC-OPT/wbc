#include "ContactsVelocityConstraint.hpp"

namespace wbc{

    void ContactsVelocityConstraint::update(RobotModelPtr robot_model) {
        
        const ActiveContacts& contacts = robot_model->getActiveContacts();

        uint nj = robot_model->noOfJoints();
        uint nc = contacts.size();
        uint nac = contacts.getNumberOfActiveContacts();

        A_mtx.resize(nac*3, nj);
        b_vec.resize(nac*3);
        b_vec.setZero();

        uint row_idx = 0;
        for(uint i = 0; i < nc; ++i){
            if(contacts[i].active){
                A_mtx.block(row_idx*3, 0, 3, nj) = robot_model->bodyJacobian(robot_model->worldFrame(), contacts.names[i]).topRows<3>();
                row_idx++;
            }
        }
    }


} // namespace wbc
