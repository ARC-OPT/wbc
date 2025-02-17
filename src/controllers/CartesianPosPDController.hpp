#ifndef WBC_CONTROLLERS_CARTPOSPDCONTROLLER_HPP
#define WBC_CONTROLLERS_CARTPOSPDCONTROLLER_HPP

#include "../types/Pose.hpp"
#include "../types/Twist.hpp"
#include "../types/SpatialAcceleration.hpp"
#include <memory>

namespace wbc {

/**
 * @brief The CartesianPosPDController class implements a PD Controller with feed forward on the RigidBodyStateSE3 type. The following control schemes are available:
 *
 * 1. Velocity Output: \f$\mathbf{v}_d = \mathbf{K}_d\mathbf{v}_r + \mathbf{K}_pe\f, \quad \| \mathbf{v}_d \| < v_{max}$
 *
 * 2. Acceleration Output: \f$\mathbf{a}_d = \mathbf{K}_{ff}\ddot{\mathbf{e}} + \mathbf{K}_d\dot{\mathbf{e}} + \mathbf{K}_p\mathbf{e}\f, \quad \| \mathbf{a}_d \| < a_{max}$,
 *
 * where pose, twist and accelerationerror are computed as
 *     \f$
 *       \renewcommand*{\arraystretch}{1.2}
 *       \mathbf{e} = \left(
 *           \begin{array}{c}
 *            \mathbf{p}_r - \mathbf{p}_a \\
 *            \mathbf{R}_a\cdot \theta \mathbf{\hat \omega}^a_r
 *            \end{array}
 *            \right)
 *       \f$
 * , \f$ \dot{\mathbf{e}} = \mathbf{v}_r-\mathbf{v} \f$ and \f$ \ddot{\mathbf{e}} = \mathbf{a}_r-\mathbf{a} \f$
 *
 * \f$ \mathbf{p},\mathbf{p}_r   \f$ - Actual and reference position <br>
 * \f$ \theta\mathbf{\hat \omega}^a_r \f$ - Rotation error as angle-axis representation <br>
 * \f$ \mathbf{R}_a \f$ - Rotation matrix, converts the rotation error to Eigen frame <br>
 * \f$ \mathbf{v},\mathbf{v}_r   \f$ - Actual and reference twist <br>
 * \f$ \mathbf{a}_r,\mathbf{a}     \f$ - Reference and actual spatial acceleration <br>
 * \f$ \mathbf{K}_d,\mathbf{K}_p \f$ - Derivative and proportial gain matrices <br>
 * \f$ \mathbf{K}_{ff} \f$ - Feed forward gain matrix <br>
 * \f$ \mathbf{a}_d,\mathbf{v}_d \f$ - Control output, desired acceleration or twist <br>
 *
 * Note: If an input is NaN, it might be ignored by the controller. E.g. if the reference or actual twist is none, the twist error will be set to zero in the controller
 */
class CartesianPosPDController {
protected:
    uint dim_controller;
    types::Twist control_out_vel;
    types::SpatialAcceleration control_out_acc;
    Eigen::VectorXd u;
    Eigen::VectorXd u_max;
    Eigen::VectorXd p_gain;
    Eigen::VectorXd d_gain;
    Eigen::VectorXd x;
    Eigen::VectorXd v;
    Eigen::VectorXd rx;
    Eigen::VectorXd rv;
    Eigen::VectorXd ra;
    types::Twist pose_diff;

public:
    /** @brief Initializes members*/
    CartesianPosPDController();
    /** @brief Compute velocity level control output*/
    const types::Twist& update(const types::Pose& ref_pose,
                               const types::Twist& ref_twist,
                               const types::Pose& pose);
    /** @brief Compute acceleration level control output*/
    const types::SpatialAcceleration& update(const types::Pose& ref_pose,
                                             const types::Twist& ref_twist,
                                             const types::SpatialAcceleration& ref_acc,
                                             const types::Pose& pose,
                                             const types::Twist& twist);
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
using CartesianPosPDControllerPtr = std::shared_ptr<CartesianPosPDController>;

}
#endif
