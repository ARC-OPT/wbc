#include "PIDController.hpp"
#include <iostream>

using namespace std;

namespace wbc{

PIDController::PIDController(uint dimension) :
    dimension(dimension){

    pid_params = PIDCtrlParams(dimension);
    control_error.setConstant(dimension, 0);
    integral.setConstant(dimension, 0);
    derivative.setConstant(dimension, 0);
    setpoint.setConstant(dimension, std::numeric_limits<double>::quiet_NaN());
    feedback.setConstant(dimension, std::numeric_limits<double>::quiet_NaN());
    control_output.setConstant(dimension, std::numeric_limits<double>::quiet_NaN());
    max_ctrl_output.setConstant(dimension, std::numeric_limits<double>::max());
    dead_zone.setConstant(dimension, 0);
    prev_control_error.setConstant(dimension, 0);
}

void PIDController::update(const double delta_t){

    // Compute control error
    control_error = setpoint - feedback;

    // Compute integral part
    integral += control_error * delta_t;

    applySaturation(integral, pid_params.windup, integral);
    applyDeadZone(control_error, dead_zone, control_error);

    derivative = computeDerivative(delta_t);

    control_output = pid_params.p_gain.cwiseProduct(control_error) +
                     pid_params.i_gain.cwiseProduct(integral) +
                     pid_params.d_gain.cwiseProduct(derivative);

    applySaturation(control_output, max_ctrl_output, control_output);
}

void PIDController::applySaturation(const Eigen::VectorXd& in, const Eigen::VectorXd& max, Eigen::VectorXd &out){
    //Apply saturation. Scale all values according to the maximum output
    double eta = 1;
    for(uint i = 0; i < in.size(); i++)
        eta = std::min( eta, max(i)/fabs(in(i)) );
    out = eta * in;
}

void PIDController::applyDeadZone(const Eigen::VectorXd& in, const Eigen::VectorXd& min, Eigen::VectorXd& out){
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
    assert(params.p_gain.size() == dimension);
    assert(params.i_gain.size() == dimension);
    assert(params.d_gain.size() == dimension);
    assert(params.windup.size() == dimension);
    pid_params = params;
}

void PIDController::setMaxCtrlOutput(const Eigen::VectorXd &max){
    assert(max.size() == dimension);
    max_ctrl_output = max;
}

void PIDController::setDeadZone(const Eigen::VectorXd &dz){
    assert(dead_zone.size() == dimension);
    dead_zone = dz;
}

const Eigen::VectorXd& PIDController::computeDerivative(const double delta_t){

    derivative = (control_error - prev_control_error)/delta_t;
    prev_control_error = control_error;

    return derivative;
}

}
