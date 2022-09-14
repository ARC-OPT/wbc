#include "RigidbodyDynamicsConstraint.hpp"

namespace wbc{

    void RigidbodyDynamicsConstraint::update(RobotModelPtr robot_model) {
        
        const ActiveContacts& contacts = robot_model->getActiveContacts();

        uint nj = robot_model->noOfJoints();
        uint na = robot_model->noOfActuatedJoints();
        uint nc = contacts.size();

        uint nv = reduced ? (nj + nc*6) : (nj + na + nc*6);

        if(reduced) // no torques in qp, consider only floating base dynamics
        {
            A_mtx.resize(6, nv);
            A_mtx.setZero();

            b_vec.resize(6);

            A_mtx.block(0,  0, 6, nj) =  robot_model->jointSpaceInertiaMatrix().topRows<6>();
            for(int i = 0; i < contacts.size(); i++)
                A_mtx.block(0, nj+i*6, 6, 6) = -robot_model->bodyJacobian(robot_model->worldFrame(), contacts.names[i]).transpose().topRows<6>();
            b_vec = -robot_model->biasForces().topRows<6>();
        }
        else
        {
            A_mtx.resize(nj, nv);
            A_mtx.setZero();
            
            b_vec.resize(nj);

            A_mtx.block(0,  0, nj, nj) =  robot_model->jointSpaceInertiaMatrix();
            A_mtx.block(0, nj, nj, na) = -robot_model->selectionMatrix().transpose();
            for(int i = 0; i < contacts.size(); i++)
                A_mtx.block(0, nj+na+i*6, nj, 6) = -robot_model->bodyJacobian(robot_model->worldFrame(), contacts.names[i]).transpose();
            b_vec = -robot_model->biasForces();
        }

    }


} // namespace wbc
