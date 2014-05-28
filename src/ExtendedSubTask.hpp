#ifndef EXTENDED_SUB_TASK_HPP
#define EXTENDED_SUB_TASK_HPP

#include "SubTask.hpp"

#include <kdl/jacobian.hpp>
#include <kdl/chainjnttojacsolver.hpp>

namespace wbc{

class ExtendedSubTask : public SubTask{
public:

    ExtendedSubTask(const SubTaskConfig& _config,
                    const uint _no_robot_joints,
                    bool _tasks_active  = true) :
        SubTask(_config, _no_robot_joints, _tasks_active)
    {
       no_task_vars = _config.task_var_names.size();
       init(no_task_vars);
       solver = 0;
    }

    ExtendedSubTask(const SubTaskConfig& _config,
                    KDL::Chain _chain,
                    const uint _no_robot_joints,
                    bool _tasks_active  = true) :
        SubTask(_config, _no_robot_joints, _tasks_active)
    {
        no_task_vars = _config.task_var_names.size();
        init(no_task_vars);

        chain = _chain;
        q.resize(chain.getNrOfJoints());
        solver = new KDL::ChainJntToJacSolver(chain);
        jac = KDL::Jacobian(chain.getNrOfJoints());
    }

    ~ExtendedSubTask(){
        if(solver)
            delete solver;
    }

    void init(uint no_vars){
        task_jac = KDL::Jacobian(no_vars);
        task_jac.data.setZero();

        H.resize(no_vars,6);
        H.setZero();

        Uf.resize(6, no_vars);
        Uf.setIdentity();

        Vf.resize(no_vars, no_vars);
        Vf.setIdentity();

        Sf.resize(no_vars);
        Sf.setZero();
    }

    double computeManipulability(const base::samples::Joints& status){
        uint index = 0;
        for(uint i = 0; i < chain.getNrOfSegments(); i++){
            KDL::Segment seg = chain.getSegment(i);
            if(seg.getJoint().getType() != KDL::Joint::None){
                double pos = status.getElementByName(seg.getJoint().getName()).position;
                q(index++) = pos;
            }
        }

        solver->JntToJac(q, jac);
        double manip = sqrt( (jac.data * jac.data.transpose()).determinant());
        return manip;
    }

    KDL::Jacobian task_jac;
    KDL::Frame pose;

    //Helpers for inversion of the task Jacobian
    Eigen::MatrixXd Uf, Vf;
    Eigen::VectorXd Sf;
    Eigen::MatrixXd H;

    //Helpers for computing manipulability
    KDL::ChainJntToJacSolver* solver;
    KDL::Chain chain;
    KDL::Jacobian jac;
    KDL::JntArray q;

};
}

#endif
