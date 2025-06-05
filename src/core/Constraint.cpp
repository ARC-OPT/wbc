#include "Constraint.hpp"

namespace wbc{

Constraint::Constraint(){

}

Constraint::Constraint(Type type)
    : c_type(type)
{
 
}

Constraint::Type Constraint::type() {
    return c_type;
} 

const Eigen::MatrixXd& Constraint::A() {
    return A_mtx;
}

const Eigen::VectorXd& Constraint::b() {
    return b_vec;
}

const Eigen::VectorXd& Constraint::lb() {
    return lb_vec;
}

const Eigen::VectorXd& Constraint::ub() {
    return ub_vec;
}

uint Constraint::size() {
    switch(c_type) {
        case Constraint::equality:
        case Constraint::inequality:
            return A_mtx.rows();
        case Constraint::bounds:
            return lb_vec.rows();
    }
    return 0;
}

}// namespace wbc
