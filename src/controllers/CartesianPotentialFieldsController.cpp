#include "CartesianPotentialFieldsController.hpp"
#include <stdexcept>

using namespace wbc;

CartesianPotentialFieldsController::CartesianPotentialFieldsController() :
    PotentialFieldsController(3){
}

const base::samples::RigidBodyStateSE3& CartesianPotentialFieldsController::update(const base::samples::RigidBodyStateSE3& feedback){

    if(!base::isnotnan(feedback.pose.position))
        throw std::runtime_error("CartesianPotentialFieldsController::update: Feedback does not have a valid position entry");
    if(p_gain.size() != dimension)
        throw std::runtime_error("CartesianPotentialFieldsController::update: PGain should have size 3, but has size " + std::to_string(p_gain.size()));

    control_output.setZero();
    for(PotentialFieldPtr f : fields){
        // Update Potential field and add gradient to control output
        control_output += f->update(feedback.pose.position);
    }
    // Multiply gain
    control_output = p_gain.cwiseProduct(control_output);

    // Apply max control output
    applySaturation(control_output, control_output);

    cartesian_control_output.time = base::Time::now();
    cartesian_control_output.twist.linear = control_output;
    cartesian_control_output.twist.angular.setZero();

    return cartesian_control_output;
}
