#include "ContactsVelocityConstraint.hpp"

namespace wbc{

    void ContactsVelocityConstraint::update(RobotModelPtr robot_model) {
        
        const std::vector<Contact>& contacts = robot_model->getContacts();

        uint nj = robot_model->nj();
        uint nc = contacts.size();
        uint nac = robot_model->nac();

        A_mtx.resize(nac*3, nj);
        b_vec.resize(nac*3);
        b_vec.setZero();

        uint row_idx = 0;
        for(uint i = 0; i < nc; ++i){
            if(contacts[i].active){
                A_mtx.block(row_idx*3, 0, 3, nj) = robot_model->spaceJacobian(contacts[i].frame_id).topRows<3>();
                row_idx++;
            }
        }
    }


} // namespace wbc
