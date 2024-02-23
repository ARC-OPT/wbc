#ifndef PID_CTRL_PARAMS_HPP
#define PID_CTRL_PARAMS_HPP

#include <base/Eigen.hpp>

namespace wbc{

class PIDCtrlParams{
public:
    PIDCtrlParams(){}
    PIDCtrlParams(uint n){
        p_gain.setConstant(n, 0);
        i_gain.setConstant(n, 0);
        d_gain.setConstant(n, 0);
        windup.setConstant(n, std::numeric_limits<double>::max());
    }
    base::VectorXd p_gain;
    base::VectorXd i_gain;
    base::VectorXd d_gain;
    base::VectorXd windup;
};
}

#endif
