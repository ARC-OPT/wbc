#ifndef WBC_CONTROLLERS_JOINTPOSPDCONTROLLER_HPP
#define WBC_CONTROLLERS_JOINTPOSPDCONTROLLER_HPP

#include <Eigen/Core>

namespace wbc {

/**
 * @brief The JointPosPDController class implements a PD Controller with feed forward on the base-Joints type. The following control schemes are available:
 *
 * 1. Velocity Output: \f$\dot{\mathbf{q}}_d = \mathbf{K}_d\dot{\mathbf{q}}_r + \mathbf{K}_pe\f$
 *
 * 2. Acceleration Output: \f$\ddot{\mathbf{q}}_d = \mathbf{K}_{ff}\ddot{\mathbf{e}} + \mathbf{K}_d\dot{\mathbf{e}} + \mathbf{K}_p\mathbf{e}\f$,
 *
 * where \f$ \mathbf{e} = \mathbf{q}_r-\mathbf{q} \f$, \f$ \dot{\mathbf{e}} = \dot{\mathbf{q}}_r-\dot{\mathbf{q}} \f$ and \f$ \ddot{\mathbf{e}} = \ddot{\mathbf{q}}_r-\ddot{\mathbf{q}} \f$
 *
 * \f$ \mathbf{q},\mathbf{q}_r   \f$ - Actual and reference joint position <br>
 * \f$ \dot{\mathbf{q}},\dot{\mathbf{q}}_r   \f$ - Actual and reference joint velocity <br>
 * \f$ \ddot{\mathbf{q}},\ddot{\mathbf{q}}_r     \f$ - Actual and reference joint acceleration <br>
 * \f$ \mathbf{K}_d,\mathbf{K}_p \f$ - Derivative and proportial gain matrices <br>
 * \f$ \mathbf{K}_{ff} \f$ - Feed forward gain matrix <br>
 * \f$ \ddot{\mathbf{q}}_d,\dot{\mathbf{q}}_d \f$ - Control output, desired acceleration or velocity <br>
 *
 * Note: If an input is NaN, it might be ignored by the controller. E.g. if the reference or actual velocity is none, the velocity error will be set to zero in the controller
 */
class JointPosPDController{
protected:
    uint dim_controller;
    Eigen::VectorXd p_gain;
    Eigen::VectorXd d_gain;
    Eigen::VectorXd u_max;
    Eigen::VectorXd u;

public:
    JointPosPDController(uint dim);

    /** @brief Compute velocity level control output*/
    const Eigen::VectorXd& update(const Eigen::VectorXd& ref_pos,
                                  const Eigen::VectorXd& ref_vel,
                                  const Eigen::VectorXd& pos);

    /** @brief Compute acceleration level control output*/
    const Eigen::VectorXd& update(const Eigen::VectorXd& ref_pos,
                                  const Eigen::VectorXd& ref_vel,
                                  const Eigen::VectorXd& ref_acc,
                                  const Eigen::VectorXd& pos,
                                  const Eigen::VectorXd& vel);

    /** Set proportional/position gain. Size has to be the same dimension of the controller*/
    void setPGain(const Eigen::VectorXd &gain);
    /** Get proportional/position gain*/
    const Eigen::VectorXd& pGain(){return p_gain;}
    /** Set derivative/velocity gain. Size has to be the same dimension of the controller*/
    void setDGain(const Eigen::VectorXd &gain);
    /** Get derivative/velocity gain*/
    const Eigen::VectorXd& dGain(){return d_gain;}
    /** Set controller saturation. Size has to be the same dimension of the controller*/
    void setMaxCtrlOutput(const Eigen::VectorXd &max_ctrl_out);
    /** Get controller saturation*/
    const Eigen::VectorXd& maxCtrlOutput(){return u_max;}
    /**
     * @brief Apply Saturation on the control output. If one or more values of <in> are bigger than the
     *        Corrresponding entry of <max>, all values will be scaled down according to the biggest
     *        ratio eta = in_i / max,i
     * @param in Input vector. Size has to be same as max.
     * @param max Maximum allowed value for input vector. Size has to be same as in.
     * @param out Output vector. Will be resized if out.size() != in.size()
     */
    void applySaturation(const Eigen::VectorXd& in, const Eigen::VectorXd& max, Eigen::VectorXd &out);
};

}
#endif // WBC_CONTROLLERS_JOINTPOSPDCONTROLLER_HPP
