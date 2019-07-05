#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include <base/Eigen.hpp>

namespace ctrl_lib{

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

class PIDController{
protected:
    PIDCtrlParams pid_params;
    uint dimension;
    base::VectorXd control_error;
    base::VectorXd dead_zone;
    base::VectorXd max_ctrl_output;
    base::VectorXd integral;
    base::VectorXd derivative;
    base::VectorXd control_output;
    base::VectorXd setpoint;
    base::VectorXd feedback;

public:
    PIDController(uint dimension);
    virtual ~PIDController(){}

    /** Compute control output*/
    void update(const double delta_t);

    void setPID(const PIDCtrlParams &params);
    const PIDCtrlParams &getPID(){return pid_params;}
    void setMaxCtrlOutput(const base::VectorXd &max);
    const base::VectorXd &getMaxCtrlOutput(){return max_ctrl_output;}
    void setDeadZone(const base::VectorXd &dz);
    const base::VectorXd &getDeadZone(){return dead_zone;}
    void applySaturation(const base::VectorXd& in, const base::VectorXd& max, base::VectorXd &out);
    void applyDeadZone(const base::VectorXd& in, const base::VectorXd& min, base::VectorXd& out);
    uint getDimension(){return dimension;}

    virtual const base::VectorXd& computeDerivative(const base::VectorXd &control_error, const double delta_t){
        // TODO
        derivative.setZero();
        return derivative;
    };
};

}

#endif
