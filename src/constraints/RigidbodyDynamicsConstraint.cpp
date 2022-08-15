#include "RigidbodyDynamicsConstraint.hpp"

namespace wbc{

    void RigidbodyDynamicsConstraint::update(RobotModelPtr robot_model) {
        
        const ActiveContacts& contacts = robot_model->getActiveContacts();

        uint nj = robot_model->noOfJoints();
        uint na = robot_model->noOfActuatedJoints();
        uint nc = contacts.size();

        uint nv = nj + na + nc*6;

        A_mtx.resize(nj, nv);
        b_vec.resize(nv);

        A_mtx.setZero();
        A_mtx.block(0,  0, nj, nj) =  robot_model->jointSpaceInertiaMatrix();
        A_mtx.block(0, nj, nj, na) = -robot_model->selectionMatrix().transpose();
        for(int i = 0; i < contacts.size(); i++)
            A_mtx.block(0, nj+na+i*6, nj, 6) = -robot_model->bodyJacobian(robot_model->worldFrame(), contacts.names[i]).transpose();
        b_vec = -robot_model->biasForces();// + robot_model->bodyJacobian(world_link, contact_link).transpose() * f_ext;

    }


} // namespace wbc
