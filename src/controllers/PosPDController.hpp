#ifndef POS_PD_CONTROLLER
#define POS_PD_CONTROLLER

#include <base/Eigen.hpp>

namespace ctrl_lib {

/**
 * @brief The PosPDController class implements the following two control scemes
 *
 *
 *      u_vel = k_d*v_ref + k_p(x_ref-x)
 *      u_acc = a_ref + k_d*(v_ref-v) + k_p(x_ref-x)
 * where
 *      x   = position
 *      v   = velocity
 *      a   = acceleration
 *      k_p = position gain
 *      k_d = velocity gain
 *      u   = Control output
 *
 *  The control output contains u1 as velocities and u2 as acceleration.
 */
class PosPDController{
protected:
    size_t dim_controller;
    base::VectorXd p_gain;
    base::VectorXd d_gain;
    base::VectorXd ff_gain;
    base::VectorXd dead_zone;
    base::VectorXd max_control_output;
    base::VectorXd ref_pos, ref_vel, ref_acc;
    base::VectorXd pos, vel;
    base::VectorXd pos_diff, vel_diff;
    base::VectorXd control_out_vel, control_out_acc;

public:
    PosPDController(size_t dim_controller);

    /** Compute control output and store it in control_out_vel and control_out_acc. Throws if any of the ref or actual position
     *  entries is NaN. Ignores NaN ref or actual velocity (disables velocity control) and NaN ref acceleration (disables acceleration feed forward)*/
    void update();
    /** Set proportional/position gain. Size has to be the same dimension of the controller*/
    const void setPGain(const base::VectorXd &gain);
    /** Get proportional/position gain*/
    const base::VectorXd& pGain(){return p_gain;}
    /** Set derivative/velocity gain. Size has to be the same dimension of the controller*/
    const void setDGain(const base::VectorXd &gain);
    /** Get derivative/velocity gain*/
    const base::VectorXd& dGain(){return d_gain;}
    /** Set feedforward gain. Size has to be the same dimension of the controller*/
    const void setFFGain(const base::VectorXd &gain);
    /** Get feedforward gain*/
    const base::VectorXd& ffGain(){return ff_gain;}
    /** Set controller saturation. Size has to be the same dimension of the controller*/
    const void setMaxCtrlOutput(const base::VectorXd &max_ctrl_out);
    /** Get controller saturation*/
    const base::VectorXd& maxCtrlOutput(){return max_control_output;}
    /** Set dead zone for the position controller. Size has to be the same dimension of the controller*/
    const void setDeadZone(const base::VectorXd &dz);
    /** Get dead zone for the position controller*/
    const base::VectorXd& deadZone(){return dead_zone;}
    /** Return position control error*/
    base::VectorXd getControlError(){return pos_diff;}
    /**
     * @brief Apply Saturation on the control output. If one or more values of <in> are bigger than the
     *        Corrresponding entry of <max>, all values will be scaled down according to the biggest
     *        ratio eta = in_i / max,i
     * @param in Input vector. Size has to be same as max.
     * @param max Maximum allowed value for input vector. Size has to be same as in.
     * @param out Output vector. Will be resized if out.size() != in.size()
     */
    void applySaturation(const base::VectorXd& in, const base::VectorXd& max, base::VectorXd &out);
    /**
     * @brief Apply dead zone, i.e. minimum position control error.
     *        If one of the input value falls below minimum, it will be set to zero. Otherwise
     *        dead zone will be subtracted from the input value, to get a smooth transition:
     *
     *        out_i = in_i - min_i if in_i >= min_i
     *              = in_i + min_i if in_i <= -min_i
     *              = 0 else
     *
     * @param in Input Vector. Size has to be same as min.
     * @param min Minimum input vector. Size has to be same as in.
     * @param out Output vector. Will be resized if out.size() != in.size()
     */
    void applyDeadZone(const base::VectorXd& in, const base::VectorXd& min, base::VectorXd& out);
};
}

#endif
