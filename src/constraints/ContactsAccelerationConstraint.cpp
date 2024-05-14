#include "ContactsAccelerationConstraint.hpp"

namespace wbc{

    void ContactsAccelerationConstraint::update(RobotModelPtr robot_model) {
        
        const ActiveContacts& contacts = robot_model->getActiveContacts();

        uint nj = robot_model->noOfJoints();
        uint na = robot_model->noOfActuatedJoints();
        uint nc = contacts.size();
        uint nac = contacts.getNumberOfActiveContacts();

        // number of optimization variables:
        // - Reduced TSID (no torques): Number robot joints nj (incl. floating base) + 6 x number of contacts nc
        // - Full TSID: Number robot joints (incl. floating base) nj + Number actuated robot joints na + 6 x number of contacts nc
        uint nv = reduced ? (nj + nc*3) : (nj + na + nc*3);

        // Constrain only contact positions (number of constraints = 3 x number of contacts), not orientations
        // (e.g. in case of point contact, the rotation is allowed to change). Also constrain only the active contacts
        A_mtx.resize(nac*3, nv);
        b_vec.resize(nac*3);

        A_mtx.setZero();
        b_vec.setZero();

        uint row_idx = 0;
        for(uint i = 0; i < contacts.size(); i++){
            if(contacts[i].active){
                const base::Acceleration& a = robot_model->spatialAccelerationBias(robot_model->worldFrame(), contacts.names[i]);
                b_vec.segment(row_idx*3, 3) = -a.linear; // use only linear part of spatial acceleration bias
                A_mtx.block(row_idx*3,  0, 3, nj) = robot_model->spaceJacobian(robot_model->worldFrame(), contacts.names[i]).topRows<3>(); // use only top 3 rows of Jacobian (only linear part)
                row_idx++;
            }
        }
    }


} // namespace wbc
