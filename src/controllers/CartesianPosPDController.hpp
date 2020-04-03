#ifndef CTRL_LIB_CART_POS_PD_CONTROLLER_HPP
#define CTRL_LIB_CART_POS_PD_CONTROLLER_HPP

#include "PosPDController.hpp"
#include <ctrl_types/RigidBodyStateSE3.hpp>

namespace ctrl_lib {

/**
 * @brief The CartesianPosPDController class implements the following 2 Control schemes:
 *
 *      u1 = k_d*v_ref + k_p(x_ref-x)
 *      u2 = a_ref + k_d(v_ref-v) + k_p(x_ref-x)
 * where
 *      a = Acceleration
 *      v = Twist
 *      x = Pose
 *      u = Control output
 *
 *  The control output contains u1 as twist and u2 as acceleration. The orientation error is computed using angle-axis notation
 *  as implemented in wbc/types/CartesianState.cpp
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
