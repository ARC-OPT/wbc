#include "WbcAccelerationScene.hpp"
#include "../core/RobotModel.hpp"
#include <base-logging/Logging.hpp>

namespace wbc{

ConstraintPtr WbcAccelerationScene::createConstraint(const ConstraintConfig &config){

    if(config.type == cart)
        return std::make_shared<CartesianAccelerationConstraint>(config, robot_model->noOfJoints());
    else if(config.type == jnt)
        return std::make_shared<JointAccelerationConstraint>(config, robot_model->noOfJoints());
    else{
        LOG_ERROR("Constraint with name %s has an invalid constraint type: %i", config.name.c_str(), config.type);
        throw std::invalid_argument("Invalid constraint config");
    }
}

void WbcAccelerationScene::update(){

    if(!configured)
        throw std::runtime_error("WBCAcceleration has not been configured!. PLease call configure() before calling update() for the first time!");

    base::samples::RigidBodyStateSE3 ref_frame;

    // Create equation system
    //    Walk through all priorities and update the optimization problem. The outcome will be
    //    A - Vector of constraint matrices. One matrix for each priority
    //    y - Vector of constraint velocities. One vector for each priority
    //    W - Vector of constraint weights. One vector for each priority
    for(uint prio = 0; prio < constraints.size(); prio++){

        constraints_prio[prio].resize(n_constraint_variables_per_prio[prio], robot_model->noOfJoints());

        // Walk through all tasks of current priority
        uint row_index = 0;
        for(uint i = 0; i < constraints[prio].size(); i++){

            constraints[prio][i]->checkTimeout();
            int type = constraints[prio][i]->config.type;
            uint n_vars = constraints[prio][i]->config.nVariables();

            if(type == cart){

                CartesianAccelerationConstraintPtr constraint = std::static_pointer_cast<CartesianAccelerationConstraint>(constraints[prio][i]);

                // Constraint Jacobian
                constraint->A = robot_model->fullJacobian(constraint->config.root, constraint->config.tip);

                // Constraint reference
                base::samples::Joints joint_state = robot_model->jointState(robot_model->jointNames());
                q_dot.resize(robot_model->noOfJoints());
                for(size_t j = 0; j < joint_state.size(); j++)
                    q_dot(j) = joint_state[j].speed;
                constraint->y_ref = constraint->y_ref - robot_model->fullJacobianDot(constraint->config.root, constraint->config.tip) * q_dot;

                // Convert input acceleration from the reference frame of the constraint to the base frame of the robot. We transform only the orientation of the
                // reference frame to which the twist is expressed, NOT the position. This means that the center of rotation for a Cartesian constraint will
                // be the origin of ref frame, not the root frame. This is more intuitive when controlling the orientation of e.g. a robot' s end effector.
                ref_frame = robot_model->rigidBodyState(constraint->config.root, constraint->config.ref_frame);
                constraint->y_ref_root.segment(0,3) = ref_frame.pose.orientation.toRotationMatrix() * constraint->y_ref.segment(0,3);
                constraint->y_ref_root.segment(3,3) = ref_frame.pose.orientation.toRotationMatrix() * constraint->y_ref.segment(3,3);

                // Also convert the weight vector from ref frame to the root frame. Take the absolute values after rotation, since weights can only
                // assume positive values
                constraint->weights_root.segment(0,3) = ref_frame.pose.orientation.toRotationMatrix() * constraint->weights.segment(0,3);
                constraint->weights_root.segment(3,3) = ref_frame.pose.orientation.toRotationMatrix() * constraint->weights.segment(3,3);
                constraint->weights_root = constraint->weights_root.cwiseAbs();

            }
            else if(type == jnt){
                JointAccelerationConstraintPtr constraint = std::static_pointer_cast<JointAccelerationConstraint>(constraints[prio][i]);

                // Joint space constraints: constraint matrix has only ones and Zeros. The joint order in the constraints might be different than in the robot model.
                // Thus, for joint space constraints, the joint indices have to be mapped correctly.
                for(uint k = 0; k < constraint->config.joint_names.size(); k++){

                    int idx = robot_model->jointIndex(constraint->config.joint_names[k]);
                    constraint->A(k,idx) = 1.0;
                    constraint->y_ref_root = constraint->y_ref;     // In joint space y_ref is equal to y_ref_root
                    constraint->weights_root = constraint->weights; // Same of the weights
                }
            }
            else{
                LOG_ERROR("Constraint %s: Invalid type: %i", constraints[prio][i]->config.name.c_str(), type);
                throw std::invalid_argument("Invalid constraint configuration");
            }

            ConstraintPtr constraint = constraints[prio][i];

            // If the activation value is zero, also set reference to zero. Activation is usually used to switch between different
            // task phases and we don't want to store the "old" reference value, in case we switch on the constraint again
            if(constraint->activation == 0){
               constraint->y_ref.setZero();
               constraint->y_ref_root.setZero();
            }

            // Insert constraints into equation system of current priority at the correct position. Note: Weights will be zero if activations
            // for this constraint is zero or if the constraint is in timeout
            constraints_prio[prio].Wy.segment(row_index, n_vars) = constraint->weights_root * constraint->activation * (!constraint->timeout);
            constraints_prio[prio].A.block(row_index, 0, n_vars, robot_model->noOfJoints()) = constraint->A;
            constraints_prio[prio].lower_y.segment(row_index, n_vars) = constraint->y_ref_root;
            constraints_prio[prio].upper_y.segment(row_index, n_vars) = constraint->y_ref_root;


            row_index += n_vars;
        }
        int nj = robot_model->noOfJoints();
        base::MatrixXd A = constraints_prio[prio].A;
        base::VectorXd y = constraints_prio[prio].lower_y;


        // Single, rigid environment contact
        // Variable order: (qdd, f, tau)
        /*constraints_prio[prio].resize(nj+6, 2*nj+6);
        constraints_prio[prio].H.setZero();
        constraints_prio[prio].H.block(0,0,nj,nj) = A.transpose()*A;
        constraints_prio[prio].g.setZero();
        constraints_prio[prio].g.segment(0,nj) = -(A.transpose()*y).transpose();

        // Constraints: [[J   0   0 ]     [qdd] =    [-Jd*qd]
        //               [M -J^T -S^T]]   [f]   =    [-h]
        //                                [tau] =
        constraints_prio[prio].A.setZero();

        // First row: Contact point left does not move
        constraints_prio[prio].A.block(0,0,6,nj)    = robot_model->fullJacobian("world", "LLAnkle_FT");
        constraints_prio[prio].lower_y.segment(0,6) = constraints_prio[prio].upper_y.segment(0,6) = -robot_model->fullJacobianDot("world", "LLAnkle_FT") * q_dot;
        // Second row: Rigid Body Dynamics
        constraints_prio[prio].A.block(6,  0,    nj, nj) =  robot_model->jointSpaceInertiaMatrix();
        constraints_prio[prio].A.block(6, nj,    nj,  6) = -robot_model->fullJacobian("world", "LLAnkle_FT").transpose();
        constraints_prio[prio].A.block(6, nj+6, nj, nj) = -robot_model->getActuationMatrix();
        constraints_prio[prio].lower_y.segment(6,nj) = constraints_prio[prio].upper_y.segment(6,nj) = -robot_model->biasForces();
        //constraints_prio[prio].lower_y.segment(6,6).setZero();
        //constraints_prio[prio].lower_y.segment(6,6).setZero();*/


        // No environment contact
        // Variable order: (qdd, tau)
        constraints_prio[prio].resize(nj, 2*nj);
        // Cost Function: x^T*H*x + x^T*
        constraints_prio[prio].H.setZero();
        constraints_prio[prio].H.block(0,0,nj,nj) = A.transpose()*A;
        constraints_prio[prio].g.setZero();
        constraints_prio[prio].g.segment(0,nj) = -(A.transpose()*y).transpose();

        // Constraints: [[M -S^T]] * [qdd tau] = [-h]
        base::Vector6d f_ext;
        f_ext << -0.0016937263217112032, -0.0031835690480501865, -75.94681724308107, -0.07960479827699948, 2.7142051814597408, -0.00011199983616109183;
        constraints_prio[prio].A.setZero();
        constraints_prio[prio].A.block(0,  0, nj, nj) =  robot_model->jointSpaceInertiaMatrix();
        constraints_prio[prio].A.block(0, nj, nj, nj) = -robot_model->getActuationMatrix();
        constraints_prio[prio].lower_y.segment(0,nj) = constraints_prio[prio].upper_y.segment(0,nj) =
                -robot_model->biasForces() - robot_model->fullJacobian("world", "LLAnkle_FT").transpose() * f_ext;
        constraints_prio[prio].lower_y.segment(0,6).setZero();
        constraints_prio[prio].upper_y.segment(0,6).setZero();

        // Lower and upper bou  nds
        constraints_prio[prio].lower_x.setConstant(-10000);
        constraints_prio[prio].upper_x.setConstant(10000);


        /*for(uint i = 0; i < robot_model->noOfActuatedJoints(); i++){
            const std::string &name = robot_model->actuatedJointNames()[i];
            try{
                constraints_prio[prio].lower_x[robot_model->jointIndex(name)+nj] = robot_model->jointLimits().getElementByName(name).min.effort;
                constraints_prio[prio].upper_x[robot_model->jointIndex(name)+nj] = robot_model->jointLimits().getElementByName(name).max.effort;
            }
            catch(base::JointLimits::InvalidName e){
                LOG_ERROR_S<<"Robot model contains joint "<<name<<" but this joint is not in joint limits vector"<<std::endl;
                throw e;
            }
        }*/

        if((base::Time::now()-stamp).toSeconds() > 1 ){
            std::cout<<"Bias: "<<robot_model->biasForces().transpose()<<std::endl;
            std::cout<<"H"<<std::endl;
            std::cout<<constraints_prio[prio].H<<std::endl;
            std::cout<<"g"<<std::endl;
            std::cout<<constraints_prio[prio].g.transpose()<<std::endl;
            std::cout<<"A"<<std::endl;
            std::cout<<constraints_prio[prio].A<<std::endl;
            std::cout<<"lower_y"<<std::endl;
            std::cout<<constraints_prio[prio].lower_y.transpose()<<std::endl;
            std::cout<<"upper_y"<<std::endl;
            std::cout<<constraints_prio[prio].upper_y.transpose()<<std::endl;
            std::cout<<"lower_x"<<std::endl;
            std::cout<<constraints_prio[prio].lower_x.transpose()<<std::endl;
            std::cout<<"upper_x"<<std::endl;
            std::cout<<constraints_prio[prio].upper_x.transpose()<<std::endl<<std::endl;
            stamp = base::Time::now();
        }
    }

    constraints_prio.joint_state = robot_model->jointState(robot_model->actuatedJointNames());
    constraints_prio.time = base::Time::now(); //  TODO: Use latest time stamp from all constraints!?
}

const ConstraintsStatus &WbcAccelerationScene::updateConstraintsStatus(const base::samples::Joints& solver_output, const base::samples::Joints& joint_state){

    if(solver_output.size() != robot_model->noOfActuatedJoints())
        throw std::runtime_error("Size of solver output is " + std::to_string(solver_output.size())
                                 + " but number of robot joints is " + std::to_string(robot_model->noOfActuatedJoints()));

    solver_output_acc.resize(robot_model->noOfJoints());
    solver_output_acc.setZero();
    for(size_t i = 0; i < solver_output.size(); i++){
        uint idx = robot_model->jointIndex(solver_output.names[i]);
        solver_output_acc(idx) = solver_output[i].acceleration;
    }
    const std::vector<std::string> &joint_names = robot_model->jointNames();
    robot_acc.resize(robot_model->noOfJoints());
    robot_vel.resize(robot_model->noOfJoints());
    for(size_t i = 0; i < joint_names.size(); i++){
        robot_acc(i) = joint_state[joint_names[i]].acceleration;
        robot_vel(i) = joint_state[joint_names[i]].speed;
    }

    for(uint prio = 0; prio < constraints.size(); prio++){
        for(uint i = 0; i < constraints[prio].size(); i++){
            ConstraintPtr constraint = constraints[prio][i];
            const std::string &name = constraint->config.name;

            constraints_status[name].time       = constraint->time;
            constraints_status[name].config     = constraint->config;
            constraints_status[name].activation = constraint->activation;
            constraints_status[name].timeout    = constraint->timeout;
            constraints_status[name].weights    = constraint->weights;
            constraints_status[name].y_ref      = constraint->y_ref;
            constraints_status[name].y_solution = robot_model->fullJacobian(constraint->config.root, constraint->config.tip) * solver_output_acc +
                                                  robot_model->fullJacobianDot(constraint->config.root, constraint->config.tip) * robot_vel;
            constraints_status[name].y          = robot_model->fullJacobian(constraint->config.root, constraint->config.tip) * robot_acc +
                                                  robot_model->fullJacobianDot(constraint->config.root, constraint->config.tip) * robot_vel;
        }
    }

    return constraints_status;
}

} // namespace wbc
