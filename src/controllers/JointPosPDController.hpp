#ifndef CTRL_LIB_JOINT_POS_PD_CONTROLLER_HPP
#define CTRL_LIB_JOINT_POS_PD_CONTROLLER_HPP

#include "PosPDController.hpp"
#include <base/commands/Joints.hpp>

namespace ctrl_lib {

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
class JointPosPDController : public PosPDController{
protected:
    base::commands::Joints control_output;
    std::vector<std::string> joint_names;

    void extractFeedback(const base::samples::Joints& feedback);
    void extractSetpoint(const base::commands::Joints& setpoint);

public:
    JointPosPDController(const std::vector<std::string>& joint_names);

    /** Convert typed to raw input data and call PosPDController::update()*/
    const base::commands::Joints& update(const base::commands::Joints& setpoint, const base::samples::Joints& feedback);
};

}
#endif
