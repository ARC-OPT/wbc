#include "TaskFrameKDL.hpp"
#include <base/Logging.hpp>
#include <kdl_conversions/KDLConversions.hpp>
#include <base/samples/Joints.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

namespace wbc{

TaskFrameKDL::TaskFrameKDL(const std::string &_name) : TaskFrame(_name){
}

TaskFrameKDL::TaskFrameKDL(const KDL::Chain& _chain, const std::string &_name) : TaskFrame(_name){

    name = _name;
    chain = _chain;
    pose_kdl = KDL::Frame::Identity();
    jacobian = KDL::Jacobian(chain.getNrOfJoints());
    jacobian.data.setZero();
    joint_positions.resize(chain.getNrOfJoints());
    joint_positions.data.setConstant(base::NaN<double>());

    for(uint i = 0; i < chain.getNrOfSegments(); i++){
        KDL::Joint joint = chain.getSegment(i).getJoint();
        if(joint.getType() != KDL::Joint::None)
            joint_names.push_back(joint.getName());
    }
}

void TaskFrameKDL::update(const base::samples::Joints &joint_state,
                          const std::vector<base::samples::RigidBodyState>& poses,
                          const std::vector<std::string> &robot_joint_names){

    //// Update joint positions
    ///
    for(uint i = 0; i < joint_names.size(); i++){
        size_t idx;
        try{
            idx = joint_state.mapNameToIndex(joint_names[i]);
        }
        catch(std::exception e){
            LOG_ERROR("Kin. Chain has joint %s, but this joint is not in joint state vector", joint_names[i].c_str());
            throw e;
        }
        if(!joint_state[idx].hasPosition()){
            LOG_ERROR("TaskFrameKDL: Position of joint %s is NaN", joint_names[i].c_str());
            throw std::runtime_error("Invalid joint position");
        }
        joint_positions(i) = joint_state[idx].position;
    }

    base::Time last_update = joint_state.time;

    //// Update segments
    ///
    KDL::Frame new_pose_kdl;
    for(size_t i = 0; i < poses.size(); i++){

        kdl_conversions::RigidBodyState2KDL(poses[i], new_pose_kdl);

        for(uint j = 0; j < chain.getNrOfSegments(); j++){

            if(chain.segments[j].getName().compare(poses[i].sourceFrame) == 0){
                chain.segments[j] = KDL::Segment(poses[i].sourceFrame,
                                                 KDL::Joint(KDL::Joint::None),
                                                 new_pose_kdl);
                last_update = poses[i].time;
            }
        }
    }

    //// Recompute Kinematics
    ///
    KDL::ChainFkSolverPos_recursive pos_fk_solver(chain);
    KDL::ChainJntToJacSolver jac_solver(chain);

    pos_fk_solver.JntToCart(joint_positions, pose_kdl);
    kdl_conversions::KDL2RigidBodyState(pose_kdl, pose);
    pose.sourceFrame = name;
    pose.time = last_update;

    //Compute Jacobian
    jac_solver.JntToJac(joint_positions, jacobian);

    // JntToJac computes Jacobian wrt root frame of the chain but with its reference point at the tip.
    // This changes the reference point to the root frame
    jacobian.changeRefPoint(-pose_kdl.p);

    // Fill in columns of task frame Jacobian into the correct place of the full robot Jacobian using the joint_index_map
    full_robot_jacobian.resize(robot_joint_names.size());
    full_robot_jacobian.data.setZero();

    for(uint j = 0; j < joint_names.size(); j++)
    {
        const std::string& jt_name =  joint_names[j];
        int idx = std::find(robot_joint_names.begin(), robot_joint_names.end(), jt_name) - robot_joint_names.begin();

        if(idx >= (int)robot_joint_names.size())
        {
            LOG_ERROR("Joint with id %s does not exist in robot joint names!", jt_name.c_str());
            throw std::invalid_argument("Invalid joint name");
        }

        full_robot_jacobian.setColumn(idx, jacobian.getColumn(j));
    }
}
}
