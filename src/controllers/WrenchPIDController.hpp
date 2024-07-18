#ifndef WBC_CONTROLLERS_WRENCHPIDCONTROLLER_HPP
#define WBC_CONTROLLERS_WRENCHPIDCONTROLLER_HPP

#include "PIDController.hpp"
#include "../types/Wrench.hpp"
#include "../types/RigidBodyState.hpp"

namespace wbc {

/**
 * @brief CartesianForcePIDController implements a PID Controller on a Wrench data type
 */
class WrenchPIDController : public PIDController{
protected:
    types::RigidBodyState control_output_wrench;

    const Eigen::VectorXd wrenchToRaw(const types::Wrench& wrench, Eigen::VectorXd& raw);

public:
    WrenchPIDController();

    /** Convert typed to raw input data and call PIDController::update()*/
    const types::RigidBodyState& update(const types::Wrench& setpoint, const types::Wrench& feedback, const double dt);
};

}

#endif // WBC_CONTROLLERS_WRENCHPIDCONTROLLER_HPP
