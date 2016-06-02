#include "WbcVelocity.hpp"
#include <kdl/utilities/svd_eigen_HH.hpp>
#include <base/logging.h>
#include "models/TaskFrameKDL.hpp"
#include "solvers/OptProblem.hpp"

using namespace std;

namespace wbc{

WbcVelocity::WbcVelocity() : Wbc(),
    configured_(false),
    temp_(Eigen::VectorXd(6)){
}

WbcVelocity::~WbcVelocity(){
    clear();
}

void WbcVelocity::clear(){

    for(uint prio = 0; prio < constraints.size(); prio++ ){
        for(uint j = 0; j < constraints[prio].size(); j++)
            delete constraints[prio][j];
        constraints[prio].clear();
    }

    jac_map_.clear();
    constraint_map_.clear();
    constraints.clear();
    joint_index_map_.clear();
}

bool WbcVelocity::configure(const std::vector<ConstraintConfig> &config,
                            const std::vector<std::string> &_joint_names){

    //Erase constraints, jacobians and joint indices
    clear();

    joint_names = _joint_names;

    // Create joint index map. This defines the order of the joints in the task Jacobians computed here. Note
    // that the order of joints in the task frames can be different.
    for(uint i = 0; i < joint_names.size(); i++)
        joint_index_map_[joint_names[i]] = i;

    // Create Constraints and sort them by priority
    int max_prio = 0;
    for(uint i = 0; i < config.size(); i++)
    {
        if(config[i].priority < 0)
        {
            LOG_ERROR("Constraint Priorities must be >= 0. Constraint priority of constraint '%s'' is %i", config[i].name.c_str(), config[i].priority);
            return false;
        }
        if(config[i].priority > max_prio)
            max_prio = config[i].priority;
    }
    constraints.resize(max_prio + 1);
    for(uint i = 0; i < config.size(); i++)
    {
        //Also put Constraints in a map that associates them with their names (for easier access)
        if(constraint_map_.count(config[i].name) != 0)
        {
            LOG_ERROR("Constraint with name %s already exists! Constraint names must be unique", config[i].name.c_str());
            return false;
        }

        ExtendedConstraint* constraint = new ExtendedConstraint(config[i], joint_names.size());
        constraints[config[i].priority].push_back(constraint);

        constraint_map_[config[i].name] = constraint;
    }

    //Erase empty priorities
    for(uint prio = 0; prio < constraints.size(); prio++)
    {
        if(constraints[prio].empty())
        {
            constraints.erase(constraints.begin() + prio, constraints.begin() + prio + 1);
            prio--;
        }
    }

    n_constraints_per_prio_ = std::vector<int>(constraints.size(), 0);
    for(uint prio = 0; prio < constraints.size(); prio++)
    {
        for(uint j = 0; j < constraints[prio].size(); j++)
            n_constraints_per_prio_[prio] += constraints[prio][j]->y_ref.size();

    }

    //Create Jacobians
    std::vector<std::string> tf_ids = getTaskFrameIDs();
    for(size_t i = 0; i < tf_ids.size(); i++){
        KDL::Jacobian jac(joint_index_map_.size());
        jac.data.setZero();
        jac_map_[tf_ids[i]] = jac;
    }

    LOG_DEBUG("Joint Index Map: \n");
    for(JointIndexMap::iterator it = joint_index_map_.begin(); it != joint_index_map_.end(); it++)
        LOG_DEBUG_S<<it->first.c_str()<<": "<<it->second;

    LOG_DEBUG("\nTask Frames: \n");
    for(size_t i = 0; i < tf_ids.size(); i++)
        LOG_DEBUG_S<<tf_ids[i];

    LOG_DEBUG("\nConstraint Map: \n");
    for(ConstraintMap::iterator it = constraint_map_.begin(); it != constraint_map_.end(); it++){
        if(it->second->config.type == cart)
            LOG_DEBUG_S<<it->first.c_str()<<": prio: "<<it->second->config.priority<<", type: "<<it->second->config.type
                      <<" root: '"<<it->second->config.root<<"'' tip: '"<<it->second->config.tip<<"'' ref frame: '"<<it->second->config.ref_frame<<"'";
        else
            LOG_DEBUG_S<<it->first.c_str()<<": prio: "<<it->second->config.priority<<", type: "<<it->second->config.type;
    }

    configured_ = true;

    return true;
}

void WbcVelocity::setupOptProblem(const std::vector<TaskFrame*> &task_frames, std::vector<OptProblem*> &opt_problem)
{
    if(!configured_)
        throw std::runtime_error("WbcVelocity::update: Configure has not been called yet");

    // Create helper map from incoming task frames
    TaskFrameMap tf_map;
    for(size_t i = 0; i < task_frames.size(); i++)
        tf_map[task_frames[i]->name] = task_frames[i];

    // Check if every required task frame is available task_frames
    std::vector<std::string> tf_ids = getTaskFrameIDs();
    for(size_t i = 0; i < tf_ids.size(); i++){

        if(tf_map.count(tf_ids[i]) == 0){
            LOG_ERROR("Wbc config requires Task Frame %s, but this task frame is not in the given task_frame vector", tf_ids[i].c_str());
            throw std::invalid_argument("Incomplete task frame vector");
        }
    }

    // Update Jacobians
    for(TaskFrameMap::const_iterator it = tf_map.begin(); it != tf_map.end(); it++)
    {
        const std::string& tf_name = it->first;
        TaskFrameKDL *tf = (TaskFrameKDL*)it->second;

        //IMPORTANT: Fill in columns of task frame Jacobian into the correct place of the full robot Jacobian using the joint_index_map
        for(uint j = 0; j < tf->joint_names.size(); j++)
        {
            const std::string& jt_name =  tf->joint_names[j];

            if(joint_index_map_.count(jt_name) == 0)
            {
                LOG_ERROR("Joint with id %s does not exist in joint index map. Check your joint_names configuration!", jt_name.c_str());
                throw std::invalid_argument("Invalid joint name");
            }

            uint idx = joint_index_map_[jt_name];
            jac_map_[tf_name].setColumn(idx, tf->jacobian.getColumn(j));
        }
    }

    if(opt_problem.size() != n_constraints_per_prio_.size())
        opt_problem.resize(n_constraints_per_prio_.size());

    //Walk through all priorities and update equation system
    for(uint prio = 0; prio < n_constraints_per_prio_.size(); prio++)
    {
        uint n_vars_prio = n_constraints_per_prio_[prio]; //Number of constraint variables on the whole priority
        ((WeightedLSSimple*)opt_problem[prio])->resize(n_vars_prio, joint_names.size());

        //Walk through all tasks of current priority
        uint row_index = 0;
        for(uint i = 0; i < constraints[prio].size(); i++)
        {
            ExtendedConstraint *constraint = (ExtendedConstraint*)constraints[prio][i];
            const uint n_vars = constraint->no_variables;

            //Check task timeout
            if(constraint->config.timeout > 0){
                double diff = (base::Time::now() - constraint->last_ref_input).toSeconds();

                if(diff > constraint->config.timeout)
                    constraint->constraint_timed_out = 1;
                else
                    constraint->constraint_timed_out = 0;
            }
            else
                constraint->constraint_timed_out = 0;

            constraint->weights_root = constraint->weights;

            if(constraint->config.type == cart)
            {
                const std::string &tf_root_name = constraint->config.root;
                const std::string &tf_tip_name = constraint->config.tip;
                const std::string &tf_ref_name = constraint->config.ref_frame;

                TaskFrameKDL* tf_root = (TaskFrameKDL*)tf_map.find(tf_root_name)->second;
                TaskFrameKDL* tf_tip = (TaskFrameKDL*)tf_map.find(tf_tip_name)->second;
                TaskFrameKDL* tf_ref_frame = (TaskFrameKDL*)tf_map.find(tf_ref_name)->second;

                //Create constraint jacobian
                constraint->pose_tip_in_root = tf_root->pose_kdl.Inverse() * tf_tip->pose_kdl;
                constraint->jac_helper.data.setIdentity();
                constraint->jac_helper.changeRefPoint(-constraint->pose_tip_in_root.p);
                constraint->jac_helper.changeRefFrame(tf_root->pose_kdl);

                //Invert constraint Jacobian
                KDL::svd_eigen_HH(constraint->jac_helper.data, constraint->Uf, constraint->Sf, constraint->Vf, temp_);

                for (unsigned int j = 0; j < constraint->Sf.size(); j++)
                {
                    if (constraint->Sf(j) > 0)
                        constraint->Uf.col(j) *= 1 / constraint->Sf(j);
                    else
                        constraint->Uf.col(j).setZero();
                }
                constraint->H = (constraint->Vf * constraint->Uf.transpose());

                const KDL::Jacobian& jac_root =  jac_map_[tf_root_name];
                const KDL::Jacobian& jac_tip =  jac_map_[tf_tip_name];

                ///// A = J^(-1) *J_tf_tip - J^(-1) * J_tf_root:
                constraint->A = constraint->H.block(0, 0, n_vars, 6) * jac_tip.data
                        -(constraint->H.block(0, 0, n_vars, 6) * jac_root.data);

                //Convert input twist from ref_frame to root of the kinematic chain
                //IMPORTANT: In KDL there are two ways to transform a twist:
                //    - KDL::Frame*KDL::Twist transforms both the reference point in which the twist is expressed AND the reference frame
                //    - KDL::Rotation*KDL::Twist transform only the reference frame!
                //    - We use KDL::Rotation*KDL::Twist here!
                //    - The difference is that with second method, after transformation, the rotational components of the twist will act like a rotation around the origin
                //      of ref_frame, expressed in the root frame of the robot. With the first method the ref_frame would rotate around the root
                //      frame (e.g. like a satellite rotates around earth), which means that rotational components, would produce translational
                //      velocities after transformation to root frame. If the twist has only translational components there is no difference between
                //      the two methods
                constraint->pose_ref_frame_in_root = tf_root->pose_kdl.Inverse() * tf_ref_frame->pose_kdl;
                KDL::Twist tmp_twist;
                for(uint i = 0; i < 6; i++)
                    tmp_twist(i) = constraint->y_ref(i);
                tmp_twist = constraint->pose_ref_frame_in_root.M * tmp_twist;
                for(uint i = 0; i < 6; i++)
                    constraint->y_ref_root(i) = tmp_twist(i);

                // Also convert the weight vector to the root frame. Weights are, as all other inputs, defined in ref_frame.
                for(uint i = 0; i < 6; i++)
                    tmp_twist(i) = constraint->weights(i);
                tmp_twist = constraint->pose_ref_frame_in_root.M * tmp_twist;
                for(uint i = 0; i < 6; i++)
                    constraint->weights_root(i) = fabs(tmp_twist(i)); //Take absolute value of weight, since weights must be positive
            }
            else if(constraint->config.type == jnt){
                for(uint i = 0; i < constraint->config.joint_names.size(); i++){

                    //Joint space constraints: constraint matrix has only ones and Zeros
                    //IMPORTANT: The joint order in the constraints might be different than in wbc.
                    //Thus, for joint space constraints, the joint indices have to be mapped correctly.
                    const std::string &joint_name = constraint->config.joint_names[i];
                    if(joint_index_map_.count(joint_name) == 0)
                    {
                        LOG_ERROR("Constraint %s contains joint %s, but this joint has not been configured in joint names", constraint->config.name.c_str(), joint_name.c_str());
                        throw std::invalid_argument("Invalid Constraint config");
                    }
                    uint idx = joint_index_map_[joint_name];
                    constraint->A(i,idx) = 1.0;
                    constraint->y_ref_root = constraint->y_ref; // In joint space, y_ref is of yourse equal to y_ref_root
                }
            }

            constraint->time = base::Time::now();

            // If the activation value is zero, also set reference to zero. Activation is usually used to switch between different
            // task phases and we don't want to store the "old" reference value, in case we switch on the constraint again
            if(constraint->activation == 0){
                constraint->y_ref.setZero();
                constraint->y_ref_root.setZero();
            }

            // Insert constraints into equation system of current priority at the correct position
            ((WeightedLSSimple*)opt_problem[prio])->W.segment(row_index, n_vars) = constraint->weights_root * constraint->activation * (!constraint->constraint_timed_out);
            ((WeightedLSSimple*)opt_problem[prio])->A.block(row_index, 0, n_vars, joint_names.size()) = constraint->A;
            ((WeightedLSSimple*)opt_problem[prio])->y_ref.segment(row_index, n_vars) = constraint->y_ref_root;

            row_index += n_vars;
        }
    }
}

void WbcVelocity::evaluateConstraints(const base::VectorXd& solver_output,
                                      const base::VectorXd& robot_vel){

    for(uint prio = 0; prio < n_constraints_per_prio_.size(); prio++)
    {
        for(uint i = 0; i < constraints[prio].size(); i++){

            ExtendedConstraint* constraint = (ExtendedConstraint*)constraints[prio][i];

            constraint->y_solution = constraint->A * solver_output;
            constraint->y = constraint->A * robot_vel;
        }
    }
}

}
