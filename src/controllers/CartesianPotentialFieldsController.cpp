#include "CartesianPotentialFieldsController.hpp"
#include <stdexcept>

using namespace wbc;

CartesianPotentialFieldsController::CartesianPotentialFieldsController() :
    PotentialFieldsController(3){
}

const types::RigidBodyState& CartesianPotentialFieldsController::update(const types::RigidBodyState& feedback){

    assert(p_gain.size() == dimension);
    assert(max_ctrl_output.size() == dimension);

    control_output.setZero();
    for(PotentialFieldPtr f : fields){
        // Update Potential field and add gradient to control output
        control_output += f->update(feedback.pose.position);
    }
    // Multiply gain
    control_output = p_gain.cwiseProduct(control_output);

    // Apply max control output
    applySaturation(control_output, control_output);

    cartesian_control_output.twist.linear = control_output;
    cartesian_control_output.twist.angular.setZero();

    return cartesian_control_output;
}
