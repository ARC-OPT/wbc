#include "PosPDController.hpp"
#include <iostream>
namespace ctrl_lib {

PosPDController::PosPDController(size_t dim_controller) :
    dim_controller(dim_controller){

    ref_pos.setConstant(dim_controller, std::numeric_limits<double>::quiet_NaN());
    ref_vel.setConstant(dim_controller, std::numeric_limits<double>::quiet_NaN());
    ref_acc.setConstant(dim_controller, std::numeric_limits<double>::quiet_NaN());
    pos.setConstant(dim_controller, std::numeric_limits<double>::quiet_NaN());
    vel.setConstant(dim_controller, std::numeric_limits<double>::quiet_NaN());
    p_gain.setConstant(dim_controller, 0);
    d_gain.setConstant(dim_controller, 0);
    ff_gain.setConstant(dim_controller, 0);
    max_control_output.setConstant(dim_controller, std::numeric_limits<double>::max());
    dead_zone.setConstant(dim_controller, 0);
    pos_diff.setConstant(dim_controller, 0);
    vel_diff.setConstant(dim_controller, 0);
    control_out_vel.setConstant(dim_controller, std::numeric_limits<double>::quiet_NaN());
    control_out_acc.setConstant(dim_controller, std::numeric_limits<double>::quiet_NaN());
}

void PosPDController::update(){
    if(!base::isnotnan(ref_pos))
        throw std::runtime_error("PosPDController::update: Setpoint position contains NaN values");
    if(!base::isnotnan(pos))
        throw std::runtime_error("PosPDController::update: Feedback position contains NaN values");

    // Compute position error
    pos_diff = ref_pos - pos;

    // Apply position dead zone
    applyDeadZone(pos_diff, dead_zone, pos_diff);

    // Multiply P-Gain
    control_out_vel = p_gain.cwiseProduct(pos_diff);

    // Add velocity Feed-forward, if it is not NaN
    if(base::isnotnan(ref_vel))
        control_out_vel += d_gain.cwiseProduct(ref_vel);

    // Compute velocity error (only if reference and actual velocity are set)
    vel_diff.setZero();
    if(base::isnotnan(ref_vel) && base::isnotnan(vel))
        vel_diff = ref_vel - vel;

    // Compute acceleration output
    control_out_acc = p_gain.cwiseProduct(pos_diff) + d_gain.cwiseProduct(vel_diff);

    // Add acceleration Feed-forward, if it is not NaN
   if(base::isnotnan(ref_acc))
       control_out_acc += ff_gain.cwiseProduct(ref_acc);

   // Apply Saturation / max. control output;
   applySaturation(control_out_vel, max_control_output, control_out_vel);
   applySaturation(control_out_acc, max_control_output, control_out_acc);
}

const void PosPDController::setPGain(const base::VectorXd &gain){
    if(!base::isnotnan(gain))
        throw std::runtime_error("PosPDController::setPGain: Invalid P-Gain. Contains NaN values");
    if(gain.size() != dim_controller)
        throw std::runtime_error("PosPDController::setPGain: Invalid P-Gain. Size is "
                                 + std::to_string(gain.size()) + " but should be " + std::to_string(dim_controller));
    p_gain = gain;
}

const void PosPDController::setDGain(const base::VectorXd &gain){
    if(!base::isnotnan(gain))
        throw std::runtime_error("PosPDController::setDGain: Invalid D-Gain. Contains NaN values");
    if(gain.size() != dim_controller)
        throw std::runtime_error("PosPDController::setPGain: Invalid D-Gain. Size is "
                                 + std::to_string(gain.size()) + " but should be " + std::to_string(dim_controller));
    d_gain = gain;
}

const void PosPDController::setFFGain(const base::VectorXd &gain){
    if(!base::isnotnan(gain))
        throw std::runtime_error("PosPDController::setFFGain: Invalid FF-Gain. Contains NaN values");
    if(gain.size() != dim_controller)
        throw std::runtime_error("PosPDController::setPGain: Invalid FF-Gain. Size is "
                                 + std::to_string(gain.size()) + " but should be " + std::to_string(dim_controller));
    ff_gain = gain;
}


const void PosPDController::setMaxCtrlOutput(const base::VectorXd &max_ctrl_out){
    if(!base::isnotnan(max_ctrl_out))
        throw std::runtime_error("PosPDController::setMaxCtrlOutput: Invalid max. control output. Contains NaN values");
    if(max_ctrl_out.size() != dim_controller)
        throw std::runtime_error("PosPDController::setPGain: Invalid max. control output. Size is "
                                 + std::to_string(max_ctrl_out.size()) + " but should be " + std::to_string(dim_controller));
    max_control_output = max_ctrl_out;
}

const void PosPDController::setDeadZone(const base::VectorXd &dz){
    if(!base::isnotnan(dead_zone))
        throw std::runtime_error("PosPDController::setDeadZone: Invalid dead zone. Contains NaN values");
    if(dead_zone.size() != dim_controller)
        throw std::runtime_error("PosPDController::setDeadZone: Invalid dead zone. Size is "
                                 + std::to_string(dead_zone.size()) + " but should be " + std::to_string(dim_controller));
    dead_zone = dz;
}

void PosPDController::applySaturation(const base::VectorXd& in, const base::VectorXd& max, base::VectorXd &out){
    //Apply saturation. Scale all values according to the maximum output
    double eta = 1;
    for(uint i = 0; i < in.size(); i++)
        eta = std::min( eta, max(i)/fabs(in(i)) );
    out = eta * in;
}

void PosPDController::applyDeadZone(const base::VectorXd& in, const base::VectorXd& min, base::VectorXd& out){
    for(uint i = 0; i < in.size(); i++){
        if(fabs(in(i)) < min(i))
            out(i) = 0.0;
        else{
            if(in(i) >= min(i))
                out(i) = in(i) - min(i);
            else if(in(i) <= -min(i))
                out(i) = in(i) + min(i);
        }
    }
}


}
