#ifndef CTRL_LIB_CART_POS_PD_CONTROLLER_HPP
#define CTRL_LIB_CART_POS_PD_CONTROLLER_HPP

#include "PosPDController.hpp"
#include <base/samples/RigidBodyStateSE3.hpp>

namespace ctrl_lib {

/**
 * @brief The CartesianPosPDController class implements a PD Controller with feed forward on the RigidBodyStateSE3 type. The following control schemes are available:
 *
 * 1. Velocity Output: \f$\mathbf{v}_d = \mathbf{K}_d\mathbf{v}_r + \mathbf{K}_pe\f$
 *
 * 2. Acceleration Output: \f$\mathbf{a}_d = \mathbf{K}_{ff}\ddot{\mathbf{e}} + \mathbf{K}_d\dot{\mathbf{e}} + \mathbf{K}_p\mathbf{e}\f$,
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
 * \f$ \mathbf{R}_a \f$ - Rotation matrix, converts the rotation error to base frame <br>
 * \f$ \mathbf{v},\mathbf{v}_r   \f$ - Actual and reference twist <br>
 * \f$ \mathbf{a}_r,\mathbf{a}     \f$ - Reference and actual spatial acceleration <br>
 * \f$ \mathbf{K}_d,\mathbf{K}_p \f$ - Derivative and proportial gain matrices <br>
 * \f$ \mathbf{K}_{ff} \f$ - Feed forward gain matrix <br>
 * \f$ \mathbf{a}_d,\mathbf{v}_d \f$ - Control output, desired acceleration or twist <br>
 *
 * Note: If an input is NaN, it might be ignored by the controller. E.g. if the reference or actual twist is none, the twist error will be set to zero in the controller
 */
class CartesianPosPDController : public PosPDController{
protected:
    base::samples::RigidBodyStateSE3 control_output;
    base::Vector6d control_error;
    base::Twist pose_diff;
    base::Acceleration vel_diff;

    void extractFeedback(const base::samples::RigidBodyStateSE3& feedback);
    void extractSetpoint(const base::samples::RigidBodyStateSE3& setpoint, const base::samples::RigidBodyStateSE3& feedback);

public:
    /** Initializes members*/
    CartesianPosPDController();
    /** Convert typed to raw input data and call PosPDController::update()*/
    const base::samples::RigidBodyStateSE3& update(const base::samples::RigidBodyStateSE3& setpoint, const base::samples::RigidBodyStateSE3& feedback);
};

}
#endif
