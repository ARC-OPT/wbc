#include "JointPosPDController.hpp"

namespace wbc{

JointPosPDController::JointPosPDController(uint dim) :
    dim_controller(dim){
    u_max.setConstant(dim,std::numeric_limits<double>::infinity());
    ff_gain.setConstant(dim,1.0);
}


const Eigen::VectorXd& JointPosPDController::update(const Eigen::VectorXd& ref_pos,
                                                    const Eigen::VectorXd& ref_vel,
                                                    const Eigen::VectorXd& pos){

    assert(ref_pos.size() == dim_controller);
    assert(ref_vel.size() == dim_controller);
    assert(pos.size() == dim_controller);
    assert(p_gain.size() == dim_controller);
    assert(ff_gain.size() == dim_controller);
    assert(u_max.size() == dim_controller);

    // Compute controller output
    u = p_gain.cwiseProduct(ref_pos - pos) + ff_gain.cwiseProduct(ref_vel);

    // Apply Saturation
    applySaturation(u, u_max, u);

    return u;
}

/** @brief Compute acceleration level control output*/
const Eigen::VectorXd& JointPosPDController::update(const Eigen::VectorXd& ref_pos,
                                                    const Eigen::VectorXd& ref_vel,
                                                    const Eigen::VectorXd& ref_acc,
                                                    const Eigen::VectorXd& pos,
                                                    const Eigen::VectorXd& vel){

    assert(ref_pos.size() == dim_controller);
    assert(ref_vel.size() == dim_controller);
    assert(ref_acc.size() == dim_controller);
    assert(pos.size() == dim_controller);
    assert(vel.size() == dim_controller);
    assert(p_gain.size() == dim_controller);
    assert(d_gain.size() == dim_controller);
    assert(ff_gain.size() == dim_controller);
    assert(u_max.size() == dim_controller);

    // Compute controller output
    u = p_gain.cwiseProduct(ref_pos - pos) + d_gain.cwiseProduct(ref_vel - vel) + ff_gain.cwiseProduct(ref_acc);

    // Apply Saturation
    applySaturation(u, u_max, u);

    return u;
}

void JointPosPDController::setPGain(const Eigen::VectorXd &gain){
    assert((size_t)gain.size() == dim_controller);
    p_gain = gain;
}

void JointPosPDController::setDGain(const Eigen::VectorXd &gain){
    assert((size_t)gain.size() == dim_controller);
    d_gain = gain;
}

void JointPosPDController::setFFGain(const Eigen::VectorXd &gain){
    assert((size_t)gain.size() == dim_controller);
    ff_gain = gain;
}

void JointPosPDController::setMaxCtrlOutput(const Eigen::VectorXd &max_ctrl_out){
    assert((size_t)max_ctrl_out.size() == dim_controller);
    u_max = max_ctrl_out;
}

void JointPosPDController::applySaturation(const Eigen::VectorXd& in, const Eigen::VectorXd& max, Eigen::VectorXd &out){
    //Apply saturation. Scale all values according to the maximum output
    double eta = 1;
    for(uint i = 0; i < in.size(); i++)
        eta = std::min( eta, max(i)/fabs(in(i)) );
    out = eta * in;
}

}
