#include "ContactsFrictionSurfaceConstraint.hpp"

namespace wbc {

void ContactsFrictionSurfaceConstraint::update(RobotModelPtr robot_model){

    const auto& contacts = robot_model->getActiveContacts();

    uint nj = robot_model->noOfJoints();
    uint na = robot_model->noOfActuatedJoints();
    uint nc = contacts.size();
    uint nac = contacts.getNumberOfActiveContacts();

    uint nv = reduced ? nj + 6*nc : nj + na + 6*nc;

    const uint row_skip = 16, col_skip = 6;

    A_mtx.resize(nac*row_skip, nv);
    lb_vec.resize(nac*row_skip);
    ub_vec.resize(nac*row_skip);

    A_mtx.setZero();
    ub_vec.setZero();
    lb_vec.setZero();

    uint start_idx = reduced ? nj : nj + na;

    uint idx = 0;
    for(uint i = 0; i < nc; i++){

        if(contacts[i].active){
            double mu=contacts[i].mu;
            double wx = contacts[i].wx, wy = contacts[i].wy;

            Eigen::MatrixXd a(row_skip,col_skip);
            a << -1,  0, -mu,  0,  0, 0,
                  1,  0, -mu,  0,  0, 0,
                  0, -1, -mu,  0,  0, 0,
                  0,  1, -mu,  0,  0, 0,
                  0,  0, -wy, -1,  0, 0,
                  0,  0, -wy,  1,  0, 0,
                  0,  0, -wx,  0, -1, 0,
                  0,  0, -wx,  0,  1, 0,
                  -wy, -wx, -(wx+wy)*mu,  mu,  mu, -1,
                  -wy,  wx, -(wx+wy)*mu,  mu, -mu, -1,
                   wy, -wx, -(wx+wy)*mu, -mu,  mu, -1,
                   wy,  wx, -(wx+wy)*mu, -mu, -mu, -1,
                   wy,  wx, -(wx+wy)*mu,  mu,  mu,  1,
                   wy, -wx, -(wx+wy)*mu,  mu, -mu,  1,
                  -wy,  wx, -(wx+wy)*mu, -mu,  mu,  1,
                  -wy, -wx, -(wx+wy)*mu, -mu, -mu,  1;

            Eigen::VectorXd lb(row_skip), ub(row_skip);
            lb.setConstant(-1e10);
            ub.setZero();

            lb_vec.segment(idx*row_skip,row_skip) = lb;
            ub_vec.segment(idx*row_skip,row_skip) = ub;
            A_mtx.block<row_skip,col_skip>(idx*row_skip,start_idx+i*6) = a;
            idx++;
        }
    }
}

}
