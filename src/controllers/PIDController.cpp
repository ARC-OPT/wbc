#include "PIDController.hpp"

using namespace std;

namespace ctrl_lib{

PIDController::PIDController(uint dimension) :
    dimension(dimension){

    pid_params = PIDCtrlParams(dimension);
    control_error.setConstant(dimension, 0);
    integral.setConstant(dimension, 0);
    derivative.setConstant(dimension, 0);
}

base::VectorXd PIDController::update(const base::VectorXd &setpoint, const base::VectorXd& feedback, const double delta_t){

    // Compute control error
    control_error = setpoint - feedback;

    // Compute integral part
    integral += control_error * delta_t;

    applySaturation(integral, pid_params.windup, integral);
    applyDeadZone(control_error, dead_zone, control_error);

    derivative = computeDerivative(control_error, delta_t);

    control_output = pid_params.p_gain.cwiseProduct(control_error) +
                     pid_params.i_gain.cwiseProduct(integral) +
                     pid_params.d_gain.cwiseProduct(derivative);

    applySaturation(control_output, max_ctrl_output, control_output);

    return control_output;
}

void PIDController::applySaturation(const base::VectorXd& in, const base::VectorXd& max, base::VectorXd &out){
    //Apply saturation. Scale all values according to the maximum output
    double eta = 1;
    for(uint i = 0; i < in.size(); i++)
        eta = std::min( eta, max(i)/fabs(in(i)) );
    out = eta * in;
}

void PIDController::applyDeadZone(const base::VectorXd& in, const base::VectorXd& min, base::VectorXd& out){
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

void PIDController::setPID(const PIDCtrlParams &params){
    if(params.p_gain.size() != dimension)
        throw runtime_error("Size of p_gain is " + to_string(params.p_gain.size()) + " but should be " + to_string(dimension));
    if(params.i_gain.size() != dimension)
        throw runtime_error("Size of i_gain is " + to_string(params.i_gain.size()) + " but should be " + to_string(dimension));
    if(params.d_gain.size() != dimension)
        throw runtime_error("Size of d_gain is " + to_string(params.d_gain.size()) + " but should be " + to_string(dimension));
    if(params.windup.size() != dimension)
        throw runtime_error("Size of windup is " + to_string(params.windup.size()) + " but should be " + to_string(dimension));
    pid_params = params;
}

void PIDController::setMaxCtrlOutput(const base::VectorXd &max){
    if(max.size() != dimension)
        throw runtime_error("Size of Max. Ctrl Output is " + to_string(max.size()) + " but should be " + to_string(dimension));
    max_ctrl_output = max;
}
void PIDController::setDeadZone(const base::VectorXd &dz){
    if(dz.size() != dimension)
        throw std::runtime_error("setDeadZone: Invalid dead zone. Size is "
                                 + std::to_string(dead_zone.size()) + " but should be " + std::to_string(dimension));
    dead_zone = dz;
}

}
