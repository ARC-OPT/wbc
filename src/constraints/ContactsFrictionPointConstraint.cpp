#include "ContactsFrictionPointConstraint.hpp"

namespace wbc {

void ContactsFrictionPointConstraint::update(RobotModelPtr robot_model){

    const bool use_torques = false;

    const auto& contacts = robot_model->getActiveContacts();

    uint nj = robot_model->noOfJoints();
    uint na = robot_model->noOfActuatedJoints();
    uint nc = contacts.size();

    uint nv = reduced ? nj + 6*nc : nj + na + 6*nc;

    const uint row_skip = use_torques ? 8 : 4;
    const uint col_skip = use_torques ? 6 : 3;

    A_mtx.resize(nc*row_skip, nv);
    lb_vec.resize(nc*row_skip);
    ub_vec.resize(nc*row_skip);

    A_mtx.setZero();
    ub_vec.setZero();
    lb_vec.setZero();

    uint start_idx = reduced ? nj : nj + na;

    for(uint i = 0; i < contacts.size(); i++){

        double mu=contacts[i].mu;

        // We assume that contact surface normal is always world_z, TODO: Make this dynamically (re-)configurable
        Eigen::MatrixXd a(row_skip,col_skip);
        a << 1,0,-mu,
             0,1,-mu,
             1,0, mu,
             0,1, mu;
        Eigen::VectorXd lb(row_skip), ub(row_skip);
        lb << -1e10,-1e10,0,0;
        ub << 0,0,1e10,1e10;

        lb_vec.segment(i*row_skip,row_skip) = lb;
        ub_vec.segment(i*row_skip,row_skip) = ub;
        A_mtx.block<row_skip,col_skip>(i*row_skip,start_idx+i*6) = a;
    }
}

}
