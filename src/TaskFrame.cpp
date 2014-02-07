#include "TaskFrame.hpp"
#include <base/logging.h>
#include <map>
#include <kdl/frames_io.hpp>

using namespace std;

namespace wbc{

TaskFrame::TaskFrame(const KDL::Chain &chain, const uint no_robot_joints){
    chain_ = chain;
    pos_fk_solver_ = new KDL::ChainFkSolverPos_recursive(chain_);
    jac_solver_ = new KDL::ChainJntToJacSolver(chain_);

    jac_ = KDL::Jacobian(chain_.getNrOfJoints());
    jac_.data.setZero(6,chain_.getNrOfJoints());

    jac_robot_ = KDL::Jacobian(no_robot_joints);
    jac_robot_.data.setZero(6,no_robot_joints);

    q_.resize(chain.getNrOfJoints());
    q_dot_.resize(chain.getNrOfJoints());
    q_dot_dot_.resize(chain.getNrOfJoints());

    for(uint i = 0; i < chain.getNrOfSegments(); i++){
        KDL::Joint joint = chain.getSegment(i).getJoint();
        if(joint.getType() != KDL::Joint::None)
            joint_names_.push_back(joint.getName());
    }

    tf_name_= chain_.segments[chain_.getNrOfSegments()-1].getName();
}

TaskFrame::~TaskFrame(){
    delete pos_fk_solver_;
    delete jac_solver_;
}

void TaskFrame::update(const base::samples::Joints &status){

    //if not done yet create joint index vector
    if(joint_index_vector_.empty()){

        //Create temporary joint index map
        std::map<std::string, uint> joint_index_map;
        for(uint i = 0; i < status.size(); i++)
            joint_index_map[status.names[i]] = i;

        for(uint i = 0; i < joint_names_.size(); i++){

            const std::string& joint_name = joint_names_[i];
            if(joint_index_map.count(joint_name) == 0){
                LOG_ERROR("No such joint in status vector: %s", joint_name.c_str());
                throw std::invalid_argument("No such joint name");
            }

            joint_index_vector_.push_back(joint_index_map[joint_name]);
        }
    }

    //Update joint variables
    for(uint i = 0; i < joint_names_.size(); i++){
        uint idx = joint_index_vector_[i];
        q_(i) = status[idx].position;
        q_dot_(i) = status[idx].speed;
        q_dot_dot_(i) = status[idx].effort;
    }

    //Compute FK
    pos_fk_solver_->JntToCart(q_, pose_);

    //Compute Jacobian
    jac_solver_->JntToJac(q_, jac_);

    // JntToJac computes Jacobian wrt root frame of the chain but with its reference point at the tip.
    // This changes the reference point to the root frame
    jac_.changeRefPoint(-pose_.p);

    //Fill in columns of Jacobian into the correct place of the full robot Jacobian
    for(uint i = 0; i < joint_names_.size(); i++){
        uint idx = joint_index_vector_[i];
        jac_robot_.setColumn(idx, jac_.getColumn(i));
    }
}

}
