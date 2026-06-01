#include "ContactsFrictionPointConstraint.hpp"

namespace wbc {

void ContactsFrictionPointConstraint::update(RobotModelPtr robot_model){

    const auto& contacts = robot_model->getContacts();

    uint nj = robot_model->nj();
    uint na = robot_model->na();
    uint nc = contacts.size();
    uint nac = robot_model->nac();

    uint nv = reduced ? nj + 3*nc : nj + na + 3*nc;

    const uint row_skip = 4;
    const uint col_skip = 3;

    A_mtx.resize(nac*row_skip, nv);
    lb_vec.resize(nac*row_skip);
    ub_vec.resize(nac*row_skip);

    A_mtx.setZero();
    ub_vec.setZero();
    lb_vec.setZero();

    uint start_idx = reduced ? nj : nj + na;
    uint idx = 0;
    for(uint i = 0; i < contacts.size(); i++){

        if(contacts[i].active){
            double mu=contacts[i].mu;

            // We assume that contact surface normal is always world_z, TODO: Make this dynamically (re-)configurable
            Eigen::MatrixXd a(row_skip,col_skip);
            a << 1,0,-mu,
                 0,1,-mu,
                 1,0, mu,
                 0,1, mu;
            Eigen::VectorXd lb(row_skip), ub(row_skip);
            lb << -1e5,-1e5,0,0;
            ub << 0,0,1e5,1e5;

            lb_vec.segment(idx*row_skip,row_skip) = lb;
            ub_vec.segment(idx*row_skip,row_skip) = ub;
            A_mtx.block<row_skip,col_skip>(idx*row_skip,start_idx+i*3) = a;
            idx++;
        }
    }
}

}
