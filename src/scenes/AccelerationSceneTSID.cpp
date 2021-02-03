#include "AccelerationSceneTSID.hpp"
#include "core/RobotModel.hpp"
#include <base-logging/Logging.hpp>

namespace wbc {

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
    uint nc = n_constraint_variables_per_prio[prio]; // total number of task constraints
    uint ncp = robot_model->getContactPoints().size();

    A.resize(nc, nj); // Task Jacobian
    y.resize(nc); // Desired task space acceleration
    wy.resize(nc); // Task weights

    // QP Size: (NJoints+NContacts*2*6 x NJoints+NActuatedJoints+NContacts*6)
    // Variable order: (acc,torque,f_ext)
    constraints_prio[prio].resize(nj+ncp*6,nj+na+ncp*6);

    // Walk through all task constraints
    uint row_index = 0;
    for(uint i = 0; i < constraints[prio].size(); i++){

        int type = constraints[prio][i]->config.type;
        constraints[prio][i]->checkTimeout();
        uint n_vars = constraints[prio][i]->config.nVariables(); // Variable for this task constraint
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

        wy.segment(row_index, n_vars) = constraint->weights_root * constraint->activation * (!constraint->timeout);
        A.block(row_index, 0, n_vars, nj) = constraint->A;
        y.segment(row_index, n_vars) = constraint->y_ref_root;
        row_index += n_vars;
    }

    // Multiply task weights
    y = wy.cwiseProduct(y);

    // Cost Function: Find joint accelerations that minimize the given task constraints (task space gradient points along desired task space accelerations)
    // Only minimize acceleration, not torques
    // min_x 0.5*x^T*H*x - 2x^T*g
    // --> H = J^T * J
    // --> g = -(J^T*xdot)^T
    constraints_prio[prio].H.setZero();
    constraints_prio[prio].H.block(0,0,nj,nj) = A.transpose()*A;
    constraints_prio[prio].g.setZero();
    constraints_prio[prio].g.segment(0,nj) = -(A.transpose()*y).transpose();

    // Task Space Constraints (for each contact point)
    constraints_prio[prio].A.setZero();
    constraints_prio[prio].lower_y.setZero();
    constraints_prio[prio].upper_y.setZero();

    // 1. M*qdd - S^T*tau - Jb_1^T*f_ext_1 - Jb_2^T*f_ext_2 - ... = -h (Rigid Body Dynamic Equation)
    std::vector<std::string> contact_points = robot_model->getContactPoints();
    constraints_prio[prio].A.block(0,  0, nj, nj) =  robot_model->jointSpaceInertiaMatrix();
    constraints_prio[prio].A.block(0, nj, nj, na) = -robot_model->selectionMatrix().transpose();
    for(int i = 0; i < contact_points.size(); i++)
        constraints_prio[prio].A.block(0, nj+na+i*6, nj, 6) = -robot_model->bodyJacobian(robot_model->baseFrame(), contact_points[i]).transpose();
    constraints_prio[prio].lower_y.segment(0,nj) = constraints_prio[prio].upper_y.segment(0,nj) = -robot_model->biasForces();// + robot_model->bodyJacobian(world_link, contact_link).transpose() * f_ext;

    // 2. For all contacts: Js*qdd = -Jsdot*qd (Rigid Contacts, contact points do not move!)
    for(int i = 0; i < contact_points.size(); i++){
        constraints_prio[prio].A.block(nj+i*6,  0, 6, nj) = robot_model->spaceJacobian(robot_model->baseFrame(), contact_points[i]);
        base::Vector6d acc;
        base::Acceleration a = robot_model->spatialAccelerationBias(robot_model->baseFrame(), contact_points[i]);
        acc.segment(0,3) = a.linear;
        acc.segment(3,3) = a.angular;
        constraints_prio[prio].lower_y.segment(nj+i*6,6) = constraints_prio[prio].upper_y.segment(nj+i*6,6) = -acc;
    }

    // Torque Limits
    constraints_prio[prio].lower_x.setConstant(-10000);
    constraints_prio[prio].upper_x.setConstant(10000);
    for(int i = 0; i < robot_model->noOfActuatedJoints(); i++){
        const std::string& name = robot_model->actuatedJointNames()[i];
        constraints_prio[prio].lower_x(i+nj) = robot_model->jointLimits()[name].min.effort;
        constraints_prio[prio].upper_x(i+nj) = robot_model->jointLimits()[name].max.effort;
    }
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

    // Convert solver output: contact wrenches
    contact_wrenches.resize(robot_model->getContactPoints().size());
    contact_wrenches.names = robot_model->getContactPoints();
    for(uint i = 0; i < robot_model->getContactPoints().size(); i++){
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
            const base::Acceleration &bias_acc = robot_model->spatialAccelerationBias(constraint->config.root, constraint->config.tip);
            const base::MatrixXd &jac = robot_model->spaceJacobian(constraint->config.root, constraint->config.tip);

            constraints_status[name].time       = constraint->time;
            constraints_status[name].config     = constraint->config;
            constraints_status[name].activation = constraint->activation;
            constraints_status[name].timeout    = constraint->timeout;
            constraints_status[name].weights    = constraint->weights;
            constraints_status[name].y_ref      = constraint->y_ref_root;
            constraints_status[name].y_solution = jac * solver_output_acc + bias_acc;
            constraints_status[name].y          = jac * robot_acc + bias_acc;
        }
    }

    return constraints_status;
}

}
