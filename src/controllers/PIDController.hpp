#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include <base/Eigen.hpp>
#include "PIDCtrlParams.hpp"

namespace wbc{

/**
 * @brief The PIDController class implements an n-dimensional PID controller
 *
 *  \f[
 *        u = k_p \cdot e + k_i\sum e \cdot dt + k_d\frac{d}{dt}e
 *  \f]
 * with integral windup
 *  \f[
 *      \sum e \cdot dt \leq w_i
 *  \f]
 * and dead zone
 * \f[
 *      e = \left \lbrace \begin{array}{ccc}x_r-x & if  & |x_r-x| > d_e \\ \\ 0 & else \end{array}\right.
 * \f]
 * \f$u\f$ - Control output<br>
 * \f$k_p,k_i,k_d\f$ - Proportional, integral and derivative gain<br>
 * \f$e\f$ - Control error<br>
 * \f$w_i\f$ - Integral windup<br>
 * \f$d_e\f$ - Dead zone<br>
 * \f$x_r,x\f$ - Reference and actual controlled value<br>
 */
class PIDController{
protected:
    PIDCtrlParams pid_params;
    uint dimension;
    base::VectorXd control_error;
    base::VectorXd prev_control_error;
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
    const base::VectorXd &maxCtrlOutput(){return max_ctrl_output;}
    void setDeadZone(const base::VectorXd &dz);
    const base::VectorXd &deadZone(){return dead_zone;}
    void applySaturation(const base::VectorXd& in, const base::VectorXd& max, base::VectorXd &out);
    void applyDeadZone(const base::VectorXd& in, const base::VectorXd& min, base::VectorXd& out);
    uint getDimension(){return dimension;}
    base::VectorXd getControlError(){return control_error;}

    virtual const base::VectorXd& computeDerivative(const double delta_t);
};

}

#endif
