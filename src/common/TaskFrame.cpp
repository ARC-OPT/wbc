#include "TaskFrame.hpp"
#include <base/Logging.hpp>
#include <kdl_conversions/KDLConversions.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

namespace wbc{

TaskFrame::TaskFrame(const KDL::Chain& _chain, const std::string &_name){

    name = _name;
    chain = _chain;
    pose = KDL::Frame::Identity();
    jacobian = KDL::Jacobian(chain.getNrOfJoints());
    jacobian.data.setZero();
    joint_positions.resize(chain.getNrOfJoints());

    for(uint i = 0; i < chain.getNrOfSegments(); i++){
        KDL::Joint joint = chain.getSegment(i).getJoint();
        if(joint.getType() != KDL::Joint::None)
            joint_names.push_back(joint.getName());
    }
}

void TaskFrame::update(const base::samples::Joints &joint_state){

    KDL::ChainFkSolverPos_recursive pos_fk_solver(chain);
    KDL::ChainJntToJacSolver jac_solver(chain);

    //Update joint variables
    for(uint i = 0; i < joint_names.size(); i++){
        size_t idx;
        try{
            idx = joint_state.mapNameToIndex(joint_names[i]);
        }
        catch(std::exception e){
            LOG_ERROR("Kin. Chain has joint %s, but this joint is not in joint state vector", joint_names[i].c_str());
            throw e;
        }
        joint_positions(i) = joint_state[idx].position;
    }

    //Compute FK
    pos_fk_solver.JntToCart(joint_positions, pose);

    //Compute Jacobian
    jac_solver.JntToJac(joint_positions, jacobian);

    // JntToJac computes Jacobian wrt root frame of the chain but with its reference point at the tip.
    // This changes the reference point to the root frame
    jacobian.changeRefPoint(-pose.p);

}

void TaskFrame::update(const base::samples::RigidBodyState &new_pose){

    KDL::Frame new_pose_kdl;
    kdl_conversions::RigidBodyState2KDL(new_pose, new_pose_kdl);

    for(uint i = 0; i < chain.getNrOfSegments(); i++)
    {
        if(chain.segments[i].getName().compare(new_pose.sourceFrame) == 0){
            chain.segments[i] = KDL::Segment(new_pose.sourceFrame,
                                             KDL::Joint(KDL::Joint::None),
                                             new_pose_kdl);
            return;
        }
        LOG_ERROR("Trying to update pose of segment %s, but this segment does not exist in cgain", new_pose.sourceFrame.c_str());
        throw std::invalid_argument("Invalid segment pose");
    }
}

const std::string& TaskFrame::rootFrame() const{
    if(chain.segments.empty())
        return name;
    return chain.getSegment(0).getName();
}

const std::string& TaskFrame::tipFrame() const{
    if(chain.segments.empty())
        return name;
    return chain.getSegment(chain.getNrOfSegments()-1).getName();
}
}
