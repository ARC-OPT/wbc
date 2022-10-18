#include "ContactsFrictionConstraint.hpp"

namespace wbc {

void ContactsFrictionConstraint::update(RobotModelPtr robot_model){

    const auto& contacts = robot_model->getActiveContacts();

    uint nj = robot_model->noOfJoints();
    uint na = robot_model->noOfActuatedJoints();
    uint nc = contacts.size();

    uint nv = reduced ? nj + 6*nc : nj + na + 6*nc;

    const uint row_skip = 8, col_skip = 3;
    A_mtx.resize(nc*row_skip, nv);
    lb_vec.resize(nc*row_skip);
    ub_vec.resize(nc*row_skip);

    A_mtx.setZero();
    ub_vec.setZero();
    lb_vec.setZero();

    uint start_idx = reduced ? nj : nj + na;
    for(int i = 0; i < contacts.size(); i++){

        double mu=contacts[i].mu;

        // ToDo: Make the following dynamically reconfigurable, currently we assume that contact surface normal is always world_z
        Eigen::Vector3d contact_normal(0,0,1);
        Eigen::Vector3d contact_tagent_x(1,0,0);
        Eigen::Vector3d contact_tagent_y(0,1,0);

        Eigen::MatrixXd a(row_skip,col_skip);
        Eigen::VectorXd lb(row_skip), ub(row_skip);
        lb.setZero();
        ub.setZero();
        a.setZero();
        a.block<1,3>(0,0) = (contact_tagent_x-mu*contact_normal).transpose();
        a.block<1,3>(1,0) = (contact_tagent_y-mu*contact_normal).transpose();
        a.block<1,3>(2,0) = (-contact_tagent_x+mu*contact_normal).transpose();
        a.block<1,3>(3,0) = (-contact_tagent_y+mu*contact_normal).transpose();
        lb[0] = -1e10;
        lb[1] = -1e10;
        ub[2] =  1e10;
        ub[3] =  1e10;

        /*const base::Vector3d &r = robot_model->rigidBodyState(robot_model->worldFrame(), contacts.names[i]).pose.position;
        a.block<1,3>(4,3) = r.cross(a.block<1,3>(0,0));
        a.block<1,3>(5,3) = r.cross(a.block<1,3>(1,0));
        a.block<1,3>(6,3) = r.cross(a.block<1,3>(2,0));
        a.block<1,3>(7,3) = r.cross(a.block<1,3>(3,0));
        lb[4] = -1e10;
        lb[5] = -1e10;
        ub[6] =  1e10;
        ub[7] =  1e10;*/

        lb_vec.segment(i*row_skip,row_skip) = lb;
        ub_vec.segment(i*row_skip,row_skip) = ub;
        A_mtx.block<row_skip,col_skip>(i*row_skip,start_idx+i*6) = a;
    }

    /*std::cout<<"A: "<<std::endl;
    std::cout<<A_mtx<<std::endl;
    std::cout<<"lb: "<<lb_vec.transpose()<<std::endl;
    std::cout<<"ub: "<<ub_vec.transpose()<<std::endl<<std::endl;*/
}

}
