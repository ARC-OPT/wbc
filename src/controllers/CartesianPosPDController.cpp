#include "CartesianPosPDController.hpp"
namespace wbc{

CartesianPosPDController::CartesianPosPDController() :
    dim_controller(6){
    rx.setConstant(6,0.0);
    rv.setConstant(6,0.0);
    ra.setConstant(6,0.0);
    x.setConstant(6,0.0);
    v.setConstant(6,0.0);
    ff_gain.setConstant(6,1.0);
    u_max.setConstant(6,std::numeric_limits<double>::infinity());
}


const types::Twist& CartesianPosPDController::update(const types::Pose& ref_pose,
                                                     const types::Twist& ref_twist,
                                                     const types::Pose& pose){

    assert(p_gain.size() == dim_controller);
    assert(ff_gain.size() == dim_controller);
    assert(u_max.size() == dim_controller);

    // Setpoint
    pose_diff = ref_pose - pose;
    rx.segment(0,3) = pose_diff.linear;
    rx.segment(3,3) = pose_diff.angular;
    rv.segment(0,3) = ref_twist.linear;
    rv.segment(3,3) = ref_twist.angular;

    // Feedback
    x.setZero();

    // Compute controller output
    u = p_gain.cwiseProduct(rx - x) + ff_gain.cwiseProduct(rv);

    // Apply Saturation
    applySaturation(u, u_max, u);

    // Convert to typed output
    control_out_vel.linear  = u.segment(0,3);
    control_out_vel.angular = u.segment(3,3);

    return control_out_vel;
}

const types::SpatialAcceleration& CartesianPosPDController::update(const types::Pose& ref_pose,
                                                                   const types::Twist& ref_twist,
                                                                   const types::SpatialAcceleration& ref_acc,
                                                                   const types::Pose& pose,
                                                                   const types::Twist& twist){    
    assert(p_gain.size() == dim_controller);
    assert(d_gain.size() == dim_controller);
    assert(ff_gain.size() == dim_controller);
    assert(u_max.size() == dim_controller);

    // Setpoint
    pose_diff = ref_pose - pose;
    rx.segment(0,3) = pose_diff.linear;
    rx.segment(3,3) = pose_diff.angular;
    rv.segment(0,3) = ref_twist.linear;
    rv.segment(3,3) = ref_twist.angular;
    ra.segment(0,3) = ref_acc.linear;
    ra.segment(3,3) = ref_acc.angular;

    // feedback
    x.setZero();
    v.segment(0,3) = twist.linear;
    v.segment(3,3) = twist.angular;

    // Compute controller output
    u = p_gain.cwiseProduct(rx - x) + d_gain.cwiseProduct(rv - v) + ff_gain.cwiseProduct(ra);

    // Apply Saturation
    applySaturation(u, u_max, u);

    // Convert to typed output
    control_out_acc.linear  = u.segment(0,3);
    control_out_acc.angular = u.segment(3,3);

    return control_out_acc;
}

void CartesianPosPDController::setPGain(const Eigen::VectorXd &gain){
    assert((size_t)gain.size() == dim_controller);
    p_gain = gain;
}

void CartesianPosPDController::setDGain(const Eigen::VectorXd &gain){
    assert((size_t)gain.size() == dim_controller);
    d_gain = gain;
}

void CartesianPosPDController::setFFGain(const Eigen::VectorXd &gain){
    assert((size_t)gain.size() == dim_controller);
    ff_gain = gain;
}

void CartesianPosPDController::setMaxCtrlOutput(const Eigen::VectorXd &max_ctrl_out){
    assert((size_t)max_ctrl_out.size() == dim_controller);
    u_max = max_ctrl_out;
}

void CartesianPosPDController::applySaturation(const Eigen::VectorXd& in, const Eigen::VectorXd& max, Eigen::VectorXd &out){
    //Apply saturation. Scale all values according to the maximum output
    double eta = 1;
    for(uint i = 0; i < in.size(); i++)
        eta = std::min( eta, max(i)/fabs(in(i)) );
    out = eta * in;
}

}
