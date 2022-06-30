#include "AccelerationSceneTSID.hpp"
#include "core/RobotModel.hpp"
#include <base-logging/Logging.hpp>

#include "../constraints/RigidbodyDynamicsHardConstraint.hpp"
#include "../constraints/ContactsAccelerationHardConstraint.hpp"
#include "../constraints/JointLimitsAccelerationHardConstraint.hpp"

namespace wbc {

AccelerationSceneTSID::AccelerationSceneTSID(RobotModelPtr robot_model, QPSolverPtr solver) :
    WbcScene(robot_model,solver),
    hessian_regularizer(1e-8){
    
    // for now manually adding constraint to this scene (an option would be to take them during configuration)
    hard_constraints.resize(1);
    hard_constraints[0].push_back(std::make_shared<RigidbodyDynamicsHardConstraint>());
    hard_constraints[0].push_back(std::make_shared<ContactsAccelerationHardConstraint>());
    hard_constraints[0].push_back(std::make_shared<JointLimitsAccelerationHardConstraint>(0.001));

}

ConstraintPtr AccelerationSceneTSID::createConstraint(const ConstraintConfig &config){

    if(config.type == cart)
        return std::make_shared<CartesianAccelerationConstraint>(config, robot_model->noOfJoints());
    else if(config.type == jnt)
        return std::make_shared<JointAccelerationConstraint>(config, robot_model->noOfJoints());
    else{
        LOG_ERROR("Constraint with name %s has an invalid constraint type: %i", config.name.c_str(), config.type);
        throw std::invalid_argument("Invalid constraint config");
    }
}

const HierarchicalQP& AccelerationSceneTSID::update(){

    if(!configured)
        throw std::runtime_error("AccelerationSceneTSID has not been configured!. PLease call configure() before calling update() for the first time!");

    if(constraints.size() != 1){
        LOG_ERROR("Number of priorities in AccelerationSceneTSID should be 1, but is %i", constraints.size());
        throw std::runtime_error("Invalid constraint configuration");
    }

    int prio = 0; // Only one priority is implemented here!
    uint nj = robot_model->noOfJoints();
    uint na = robot_model->noOfActuatedJoints();
    uint ncp = robot_model->getActiveContacts().size();

    // QP Size: (NJoints+NContacts*2*6 x NJoints+NActuatedJoints+NContacts*6)
    // Variable order: (acc,torque,f_ext)
    constraints_prio[prio].resize(nj+ncp*6,nj+na+ncp*6);
    constraints_prio[prio].H.setZero();
    constraints_prio[prio].g.setZero();

    ///////// Tasks

    // Walk through all tasks
    for(uint i = 0; i < constraints[prio].size(); i++){

        int type = constraints[prio][i]->config.type;
        constraints[prio][i]->checkTimeout();
        ConstraintPtr constraint;

        if(type == cart){
            constraint = std::static_pointer_cast<CartesianAccelerationConstraint>(constraints[prio][i]);

            // Task Jacobian
            constraint->A = robot_model->spaceJacobian(constraint->config.root, constraint->config.tip);

             // Desired task space acceleration: y_r = y_d - Jdot*qdot
            base::samples::Joints joint_state = robot_model->jointState(robot_model->jointNames());
            q_dot.resize(robot_model->noOfJoints());
            for(size_t j = 0; j < joint_state.size(); j++)
                q_dot(j) = joint_state[j].speed;
            constraint->y_ref = constraint->y_ref - robot_model->spatialAccelerationBias(constraint->config.root, constraint->config.tip);

            // Convert input acceleration from the reference frame of the constraint to the base frame of the robot. We transform only the orientation of the
            // reference frame to which the twist is expressed, NOT the position. This means that the center of rotation for a Cartesian constraint will
            // be the origin of ref frame, not the root frame. This is more intuitive when controlling the orientation of e.g. a robot' s end effector.
            base::samples::RigidBodyStateSE3 ref_frame = robot_model->rigidBodyState(constraint->config.root, constraint->config.ref_frame);
            constraint->y_ref_root.segment(0,3) = ref_frame.pose.orientation.toRotationMatrix() * constraint->y_ref.segment(0,3);
            constraint->y_ref_root.segment(3,3) = ref_frame.pose.orientation.toRotationMatrix() * constraint->y_ref.segment(3,3);

            // Also convert the weight vector from ref frame to the root frame. Take the absolute values after rotation, since weights can only
            // assume positive values
            constraint->weights_root.segment(0,3) = ref_frame.pose.orientation.toRotationMatrix() * constraint->weights.segment(0,3);
            constraint->weights_root.segment(3,3) = ref_frame.pose.orientation.toRotationMatrix() * constraint->weights.segment(3,3);
            constraint->weights_root = constraint->weights_root.cwiseAbs();
        }
        else if(type == jnt){
            constraint = std::static_pointer_cast<JointAccelerationConstraint>(constraints[prio][i]);

            // Joint space constraints: constraint matrix has only ones and Zeros. The joint order in the constraints might be different than in the robot model.
            // Thus, for joint space constraints, the joint indices have to be mapped correctly.
            for(uint k = 0; k < constraint->config.joint_names.size(); k++){

                int idx = robot_model->jointIndex(constraint->config.joint_names[k]);
                constraint->A(k,idx) = 1.0;
                constraint->y_ref_root = constraint->y_ref;     // In joint space y_ref is equal to y_ref_root
                constraint->weights_root = constraint->weights; // Same for the weights
            }

        }
        else{
            LOG_ERROR("Constraint %s: Invalid type: %i", constraints[prio][i]->config.name.c_str(), type);
            throw std::invalid_argument("Invalid constraint configuration");
        }

        // If the activation value is zero, also set reference to zero. Activation is usually used to switch between different
        // task phases and we don't want to store the "old" reference value, in case we switch on the constraint again
        if(constraint->activation == 0){
           constraint->y_ref.setZero();
           constraint->y_ref_root.setZero();
        }

        for(int i = 0; i < constraint->A.rows(); i++)
            constraint->Aw.row(i) = constraint->weights_root(i) * constraint->A.row(i) * constraint->activation * (!constraint->timeout);
        for(int i = 0; i < constraint->A.cols(); i++)
            constraint->Aw.col(i) = joint_weights[i] * constraint->Aw.col(i);

        constraints_prio[prio].H.block(0,0,nj,nj) += constraint->Aw.transpose()*constraint->Aw;
        constraints_prio[prio].g.segment(0,nj) -= constraint->Aw.transpose()*constraint->y_ref_root;
    }

    constraints_prio[prio].H.block(0,0,nj,nj).diagonal().array() += hessian_regularizer;


    ///////// Constraints

    size_t total_eqs = 0;
    for(auto hard_contraint : hard_constraints[prio]) {
        hard_contraint->update(robot_model);
        if(hard_contraint->type() != HardConstraint::bounds)
            total_eqs += hard_contraint->size();
    }

    // Note already performed at the beginning of the update (but does not consider additional constriants)
    constraints_prio[prio].A.resize(total_eqs, nj+na+ncp*6);
    constraints_prio[prio].lower_x.resize(nj+na+ncp*6);
    constraints_prio[prio].upper_y.resize(nj+na+ncp*6);
    constraints_prio[prio].lower_y.resize(total_eqs);
    constraints_prio[prio].upper_y.resize(total_eqs);

    constraints_prio[prio].A.setZero();
    constraints_prio[prio].lower_y.setConstant(-99999);
    constraints_prio[prio].upper_y.setConstant(+99999);
    constraints_prio[prio].lower_x.setConstant(-99999);
    constraints_prio[prio].upper_x.setConstant(+99999);

    total_eqs = 0;
    for(uint i = 0; i < hard_constraints[prio].size(); i++) {
        HardConstraint::Type type = hard_constraints[prio][i]->type();
        size_t c_size = hard_constraints[prio][i]->size();

        if(type == HardConstraint::bounds) {
            constraints_prio[prio].lower_x = hard_constraints[prio][i]->lb();
            constraints_prio[prio].upper_x = hard_constraints[prio][i]->ub();
        }
        else if (type == HardConstraint::equality) {
            constraints_prio[prio].A.middleRows(total_eqs, c_size) = hard_constraints[prio][i]->A();
            constraints_prio[prio].lower_y.segment(total_eqs, c_size) = hard_constraints[prio][i]->b();
            constraints_prio[prio].upper_y.segment(total_eqs, c_size) = hard_constraints[prio][i]->b();
        }
        else if (type == HardConstraint::inequality) {
            constraints_prio[prio].A.middleRows(total_eqs, c_size) = hard_constraints[prio][i]->A();
            constraints_prio[prio].lower_y.segment(total_eqs, c_size) = hard_constraints[prio][i]->lb();
            constraints_prio[prio].upper_y.segment(total_eqs, c_size) = hard_constraints[prio][i]->ub();
        }

        total_eqs += c_size;
    }
    
    // // 1. M*qdd - S^T*tau - Jb_1^T*f_ext_1 - Jb_2^T*f_ext_2 - ... = -h (Rigid Body Dynamic Equation)

    // ActiveContacts contact_points = robot_model->getActiveContacts();
    // constraints_prio[prio].A.block(0,  0, nj, nj) =  robot_model->jointSpaceInertiaMatrix();
    // constraints_prio[prio].A.block(0, nj, nj, na) = -robot_model->selectionMatrix().transpose();
    // for(int i = 0; i < contact_points.size(); i++)
    //     constraints_prio[prio].A.block(0, nj+na+i*6, nj, 6) = -robot_model->bodyJacobian(robot_model->baseFrame(), contact_points.names[i]).transpose();
    // constraints_prio[prio].lower_y.segment(0,nj) = constraints_prio[prio].upper_y.segment(0,nj) = -robot_model->biasForces();// + robot_model->bodyJacobian(world_link, contact_link).transpose() * f_ext;

    // // 2. For all contacts: Js*qdd = -Jsdot*qd (Rigid Contacts, contact points do not move!)

    // for(int i = 0; i < contact_points.size(); i++){
    //     constraints_prio[prio].A.block(nj+i*6,  0, 6, nj) = robot_model->spaceJacobian(robot_model->baseFrame(), contact_points.names[i]);
    //     base::Vector6d acc;
    //     base::Acceleration a = robot_model->spatialAccelerationBias(robot_model->baseFrame(), contact_points.names[i]);
    //     acc.segment(0,3) = a.linear;
    //     acc.segment(3,3) = a.angular;
    //     constraints_prio[prio].lower_y.segment(nj+i*6,6) = constraints_prio[prio].upper_y.segment(nj+i*6,6) = -acc;
    // }

    // // 3. Torque and acceleration limits

    // constraints_prio[prio].upper_x.setConstant(10000);
    // constraints_prio[prio].lower_x.setConstant(-10000);
    // for(int i = 0; i < robot_model->noOfActuatedJoints(); i++){
    //     const std::string& name = robot_model->actuatedJointNames()[i];
    //     constraints_prio[prio].lower_x(i+nj) = robot_model->jointLimits()[name].min.effort;
    //     constraints_prio[prio].upper_x(i+nj) = robot_model->jointLimits()[name].max.effort;
    // }

    constraints_prio.Wq = base::VectorXd::Map(joint_weights.elements.data(), robot_model->noOfJoints());
    constraints_prio.time = base::Time::now(); //  TODO: Use latest time stamp from all constraints!?
    return constraints_prio;
}

const base::commands::Joints& AccelerationSceneTSID::solve(const HierarchicalQP& hqp){

    // solve
    solver_output.resize(hqp[0].nq);
    solver->solve(hqp, solver_output);

    // Convert solver output: Acceleration and torque
    uint nj = robot_model->noOfJoints();
    uint na = robot_model->noOfActuatedJoints();
    solver_output_joints.resize(robot_model->noOfActuatedJoints());
    solver_output_joints.names = robot_model->actuatedJointNames();
    for(uint i = 0; i < robot_model->noOfActuatedJoints(); i++){
        const std::string& name = robot_model->actuatedJointNames()[i];
        uint idx = robot_model->jointIndex(name);
        solver_output_joints[name].acceleration = solver_output[idx];
        solver_output_joints[name].effort = solver_output[i+nj];
    }
    solver_output_joints.time = base::Time::now();

    /*std::cout<<"Acc:   "<<solver_output.segment(0,nj).transpose()<<std::endl;
    std::cout<<"Tau:   "<<solver_output.segment(nj,na).transpose()<<std::endl;
    std::cout<<"F_ext: "<<solver_output.segment(nj+na,12).transpose()<<std::endl<<std::endl;*/

    // Convert solver output: contact wrenches
    contact_wrenches.resize(robot_model->getActiveContacts().size());
    contact_wrenches.names = robot_model->getActiveContacts().names;
    for(uint i = 0; i < robot_model->getActiveContacts().size(); i++){
        contact_wrenches[i].force = solver_output.segment(nj+na+i*6,3);
        contact_wrenches[i].torque = solver_output.segment(nj+na+i*6+3,3);
    }

    contact_wrenches.time = base::Time::now();
    return solver_output_joints;
}

const ConstraintsStatus& AccelerationSceneTSID::updateConstraintsStatus(){

    uint nj = robot_model->noOfJoints();
    solver_output_acc = solver_output.segment(0,nj);
    const base::samples::Joints& joint_state = robot_model->jointState(robot_model->jointNames());
    robot_acc.resize(nj);
    for(size_t i = 0; i < nj; i++)
        robot_acc(i) = joint_state[i].acceleration;

    for(uint prio = 0; prio < constraints.size(); prio++){
        for(uint i = 0; i < constraints[prio].size(); i++){
            ConstraintPtr constraint = constraints[prio][i];
            const std::string &name = constraint->config.name;

            constraints_status[name].time       = constraint->time;
            constraints_status[name].config     = constraint->config;
            constraints_status[name].activation = constraint->activation;
            constraints_status[name].timeout    = constraint->timeout;
            constraints_status[name].weights    = constraint->weights;
            constraints_status[name].y_ref      = constraint->y_ref_root;
            if(constraint->config.type == cart){
                const base::MatrixXd &jac = robot_model->spaceJacobian(constraint->config.root, constraint->config.tip);
                const base::Acceleration &bias_acc = robot_model->spatialAccelerationBias(constraint->config.root, constraint->config.tip);
                constraints_status[name].y_solution = jac * solver_output_acc + bias_acc;
                constraints_status[name].y          = jac * robot_acc + bias_acc;
            }
        }
    }

    return constraints_status;
}

}
