#include "WrenchForwardTask.hpp"
#include <base-logging/Logging.hpp>

namespace wbc {

WrenchForwardTask::WrenchForwardTask(TaskConfig config, uint n_robot_joints) :
    CartesianTask(config, n_robot_joints){

}

void WrenchForwardTask::update(RobotModelPtr robot_model){

    // Task Jacobian is identity here: The external wrenches are explicit variables in the QP,
    // so the reference wrench is simply forward to these variables
    A = Eigen::MatrixXd::Identity(config.nVariables(),config.nVariables());
    Aw = Eigen::MatrixXd::Identity(config.nVariables(),config.nVariables());

    this->y_ref_root = y_ref;

    // Also convert the weight vector from ref frame to the root frame. Take the absolute values after rotation, since weights can only
    // assume positive values

    ref_frame_pose = robot_model->rigidBodyState(config.root, config.ref_frame).pose;
    ref_frame_rotation = ref_frame_pose.orientation.toRotationMatrix();
    weights_root.segment(0,3) = ref_frame_rotation * weights.segment(0,3);
    weights_root.segment(3,3) = ref_frame_rotation * weights.segment(3,3);
    weights_root = weights_root.cwiseAbs();
}

void WrenchForwardTask::setReference(const base::samples::RigidBodyStateSE3& ref){

    if(!ref.hasValidWrench()){
        LOG_ERROR("Task %s has invalid force and/or torque", config.name.c_str())
        throw std::invalid_argument("Invalid task reference value");
    }

    if(ref.time.isNull())
        this->time = base::Time::now();
    else
        this->time = ref.time;

    this->y_ref.segment(0,3) = ref.wrench.force;
    this->y_ref.segment(3,3) = ref.wrench.torque;
}


}
