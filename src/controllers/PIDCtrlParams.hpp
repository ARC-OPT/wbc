#ifndef PID_CTRL_PARAMS_HPP
#define PID_CTRL_PARAMS_HPP

#include <Eigen/Core>

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
    Eigen::VectorXd p_gain;
    Eigen::VectorXd i_gain;
    Eigen::VectorXd d_gain;
    Eigen::VectorXd windup;
};
}

#endif
