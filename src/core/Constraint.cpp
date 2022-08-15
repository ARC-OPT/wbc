#include "Constraint.hpp"
#include <base-logging/Logging.hpp>
#include <base/Float.hpp>

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

const base::MatrixXd& Constraint::A() {
    return A_mtx;
}

const base::VectorXd& Constraint::b() {
    return b_vec;
}

const base::VectorXd& Constraint::lb() {
    return lb_vec;
}

const base::VectorXd& Constraint::ub() {
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

// void Constraint::setMask(const base::VectorXi& mask){
//     if(config.nVariables() != mask.size()){
//         LOG_ERROR("Constraint %s: Size of mask vector should be %i but is %i", config.name.c_str(), config.nVariables(), weights.size())
//         throw std::invalid_argument("Invalid constraint weights");
//     }

//     for(uint i = 0; i < weights.size(); i++)
//         if(weights(i) != 0 || weights(i) != 1){
//             LOG_ERROR("Constraint %s: Mask values should be among {0 ,1}, but mask %i is %d", config.name.c_str(), i, weights(i));
//             throw std::invalid_argument("Invalid constraint mask");
//         }

//     this->mask = mask;
// }



}// namespace wbc
