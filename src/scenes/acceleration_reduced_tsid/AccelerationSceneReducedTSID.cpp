#include "AccelerationSceneReducedTSID.hpp"
#include "core/RobotModel.hpp"
#include <base-logging/Logging.hpp>

#include "../../tasks/JointAccelerationTask.hpp"
#include "../../tasks/CartesianAccelerationTask.hpp"
#include "../../tasks/CoMAccelerationTask.hpp"
#include "../../tasks/WrenchForwardTask.hpp"

#include "../../constraints/RigidbodyDynamicsConstraint.hpp"
#include "../../constraints/ContactsAccelerationConstraint.hpp"
#include "../../constraints/JointLimitsAccelerationConstraint.hpp"
#include "../../constraints/EffortLimitsAccelerationConstraint.hpp"
#include "../../constraints/ContactsFrictionSurfaceConstraint.hpp"


namespace wbc {

SceneRegistry<AccelerationSceneReducedTSID> AccelerationSceneReducedTSID::reg("acceleration_reduced_tsid");

AccelerationSceneReducedTSID::AccelerationSceneReducedTSID(RobotModelPtr robot_model, QPSolverPtr solver, const double dt) :
    Scene(robot_model, solver, dt),
    hessian_regularizer(1e-8){

    // whether or not torques are removed  from the qp problem
    // this formulation includes torques !!!
    bool reduced = true; // DO NOT CHANGE

    // for now manually adding constraint to this scene (an option would be to take them during configuration)
    constraints.resize(1);
    constraints[0].push_back(std::make_shared<RigidbodyDynamicsConstraint>(reduced));
    constraints[0].push_back(std::make_shared<ContactsAccelerationConstraint>(reduced));
    constraints[0].push_back(std::make_shared<JointLimitsAccelerationConstraint>(dt, reduced));
    constraints[0].push_back(std::make_shared<EffortLimitsAccelerationConstraint>());
    constraints[0].push_back(std::make_shared<ContactsFrictionSurfaceConstraint>(reduced));
}

TaskPtr AccelerationSceneReducedTSID::createTask(const TaskConfig &config){

    if(config.type == cart)
        return std::make_shared<CartesianAccelerationTask>(config, robot_model->noOfJoints());
    else if(config.type == com)
        return std::make_shared<CoMAccelerationTask>(config, robot_model->noOfJoints());
    else if(config.type == jnt)
        return std::make_shared<JointAccelerationTask>(config, robot_model->noOfJoints());
    else if(config.type == wrench_forward)
        return std::make_shared<WrenchForwardTask>(config, robot_model->noOfJoints());
    else{
        LOG_ERROR("Task with name %s has an invalid task type: %i", config.name.c_str(), config.type);
        throw std::invalid_argument("Invalid task config");
    }
}

const HierarchicalQP& AccelerationSceneReducedTSID::update(){

    if(!configured)
        throw std::runtime_error("AccelerationSceneReducedTSID has not been configured!. PLease call configure() before calling update() for the first time!");

    if(tasks.size() != 1){
        LOG_ERROR("Number of priorities in AccelerationSceneReducedTSID should be 1, but is %i", tasks.size());
        throw std::runtime_error("Invalid task configuration");
    }

    int prio = 0; // Only one priority is implemented here!
    uint nj = robot_model->noOfJoints();
    uint ncp = robot_model->getActiveContacts().size();

    //////// Constraints

    bool has_bounds = false;
    size_t total_eqs = 0, total_ineqs = 0;
    for(auto contraint : constraints[prio]) {
        contraint->update(robot_model);
        if(contraint->type() == Constraint::equality)
            total_eqs += contraint->size();
        if(contraint->type() == Constraint::inequality)
            total_ineqs += contraint->size();
        if(contraint->type() == Constraint::bounds)
            has_bounds = true;
    }

    // QP Size: (nc x nj+nc*6)
    // Variable order: (qdd,f_ext)
    QuadraticProgram& qp = hqp[prio];
    qp.resize(nj+ncp*6, total_eqs, total_ineqs, has_bounds);
    qp.A.setZero();
    qp.lower_x.setConstant(-10000);   // bounds
    qp.upper_x.setConstant(+10000);   // bounds
    qp.lower_y.setZero(); // inequalities
    qp.upper_y.setZero(); // inequalities

    total_eqs = 0, total_ineqs = 0;
    for(uint i = 0; i < constraints[prio].size(); i++) {
        Constraint::Type type = constraints[prio][i]->type();
        size_t c_size = constraints[prio][i]->size();

        if(type == Constraint::bounds) {
            qp.lower_x = constraints[prio][i]->lb(); // NOTE! Good only if a single bound task is admitted
            qp.upper_x = constraints[prio][i]->ub();
        }
        else if (type == Constraint::equality) {
            qp.A.middleRows(total_eqs, c_size) = constraints[prio][i]->A();
            qp.b.segment(total_eqs, c_size) = constraints[prio][i]->b();
            total_eqs += c_size;
        }
        else if (type == Constraint::inequality) {
            qp.C.middleRows(total_ineqs, c_size) = constraints[prio][i]->A();
            qp.lower_y.segment(total_ineqs, c_size) = constraints[prio][i]->lb();
            qp.upper_y.segment(total_ineqs, c_size) = constraints[prio][i]->ub();
            total_ineqs += c_size;
        }
    }

    ///////// Tasks

    qp.H.setZero();
    qp.g.setZero();
    for(uint i = 0; i < tasks[prio].size(); i++){
        
        TaskPtr task = tasks[prio][i];

        task->checkTimeout();
        task->update(robot_model);

        // If the activation value is zero, also set reference to zero. Activation is usually used to switch between different
        // task phases and we don't want to store the "old" reference value, in case we switch on the task again
        if(task->activation == 0){
           task->y_ref.setZero();
           task->y_ref_root.setZero();
        }

        for(int i = 0; i < task->A.rows(); i++){
            task->Aw.row(i) = task->weights_root(i) * task->A.row(i) * task->activation * (!task->timeout);
            task->y_ref_root(i) = task->y_ref_root(i) * task->weights_root(i) * task->activation;
        }
        for(int i = 0; i < task->A.cols(); i++)
            task->Aw.col(i) = joint_weights[i] * task->Aw.col(i);

        // Decide on which output variables the tasks are to be mapped: qdd or f_ext
        if(task->config.type == wrench_forward){ // f_ext
            const ActiveContacts &cp = robot_model->getActiveContacts();
            const std::string contact_link = task->config.ref_frame;
            if(std::find(cp.names.begin(), cp.names.end(), contact_link) == cp.names.end()){
                LOG_ERROR("Reference frame defined in wrench task %s must match one of the contact points defined robot model", task->config.name.c_str());
                LOG_ERROR("You set reference frame %s, but this frame is not a contact point", contact_link.c_str());
                throw std::runtime_error("Invalid task configuration");
            }
            uint idx = cp.mapNameToIndex(contact_link);
            qp.H.block(nj+idx*6,nj+idx*6,6,6) += task->Aw.transpose()*task->Aw;
            qp.g.segment(nj+idx*6,6) -= task->Aw.transpose()*task->y_ref_root;
        }
        else{ // qdd
            qp.H.block(0,0,nj,nj) += task->Aw.transpose()*task->Aw;
            qp.g.segment(0,nj) -= task->Aw.transpose()*task->y_ref_root;
        }
    }

    qp.H.block(0,0, nj, nj).diagonal().array() += hessian_regularizer;
    qp.H.block(nj,nj, ncp*6, ncp*6).diagonal().array() += 1e-12;

    hqp.Wq = base::VectorXd::Map(joint_weights.elements.data(), robot_model->noOfJoints());
    hqp.time = base::Time::now(); //  TODO: Use latest time stamp from all tasks!?
    return hqp;
}

const base::commands::Joints& AccelerationSceneReducedTSID::solve(const HierarchicalQP& hqp){

    // solve
    solver_output.resize(hqp[0].nq);
    solver->solve(hqp, solver_output);

    const auto& contacts = robot_model->getActiveContacts();

    // Convert solver output: Acceleration and torque
    uint nj = robot_model->noOfJoints();
    uint na = robot_model->noOfActuatedJoints();
    uint nc = contacts.size();

    auto qdd_out = Eigen::Map<Eigen::VectorXd>(solver_output.data(), nj);
    auto fext_out = Eigen::Map<Eigen::VectorXd>(solver_output.data()+nj, 6*nc);

    // computing torques from accelerations and forces (using last na equation from dynamic equations of motion)
    Eigen::VectorXd tau_out = robot_model->jointSpaceInertiaMatrix().bottomRows(na) * qdd_out;
    for(uint c = 0; c < nc; ++c)
        tau_out += -robot_model->bodyJacobian(robot_model->worldFrame(), contacts.names[c]).transpose().bottomRows(na) * fext_out.segment<6>(c*6);
    tau_out += robot_model->biasForces().bottomRows(na);

    solver_output_joints.resize(robot_model->noOfActuatedJoints());
    solver_output_joints.names = robot_model->actuatedJointNames();
    for(uint i = 0; i < robot_model->noOfActuatedJoints(); i++){
        const std::string& name = robot_model->actuatedJointNames()[i];
        uint idx = robot_model->jointIndex(name);
        if(base::isNaN(qdd_out[idx])){
            hqp[0].print();
            throw std::runtime_error("Solver output (acceleration) for joint " + name + " is NaN");
        }
        uint start_idx = robot_model->hasFloatingBase() ? 6 : 0;
        if(base::isNaN(tau_out[idx-start_idx])){
            hqp[0].print();
            throw std::runtime_error("Solver output (force/torque) for joint " + name + " is NaN");
        }
        solver_output_joints[name].acceleration = qdd_out[idx];
        solver_output_joints[name].effort = tau_out[idx-start_idx]; // tau_out does not include fb dofs.
    }
    solver_output_joints.time = base::Time::now();

    // std::cerr << "Acc:   " << qdd_out.transpose() << std::endl;
    // std::cerr << "Tau:   " << tau_out.transpose() << std::endl;
    // std::cerr << "F_ext: " << fext_out.transpose() << std::endl << std::endl;

    // Convert solver output: contact wrenches
    contact_wrenches.resize(nc);
    contact_wrenches.names = contacts.names;
    for(uint i = 0; i < nc; i++){
        contact_wrenches[i].force = fext_out.segment(i*6, 3);
        contact_wrenches[i].torque = fext_out.segment(i*6+3, 3);
    }

    contact_wrenches.time = base::Time::now();
    return solver_output_joints;
}

const TasksStatus& AccelerationSceneReducedTSID::updateTasksStatus(){

    uint nj = robot_model->noOfJoints();
    solver_output_acc = solver_output.segment(0,nj);
    const base::samples::Joints& joint_state = robot_model->jointState(robot_model->jointNames());
    robot_acc.resize(nj);
    for(size_t i = 0; i < nj; i++)
        robot_acc(i) = joint_state[i].acceleration;

    for(uint prio = 0; prio < tasks.size(); prio++){
        for(uint i = 0; i < tasks[prio].size(); i++){
            TaskPtr task = tasks[prio][i];
            const std::string &name = task->config.name;

            tasks_status[name].time       = task->time;
            tasks_status[name].config     = task->config;
            tasks_status[name].activation = task->activation;
            tasks_status[name].timeout    = task->timeout;
            tasks_status[name].weights    = task->weights;
            tasks_status[name].y_ref      = task->y_ref_root;
            if(task->config.type == cart){
                const base::MatrixXd &jac = robot_model->spaceJacobian(task->config.root, task->config.tip);
                const base::Acceleration &bias_acc = robot_model->spatialAccelerationBias(task->config.root, task->config.tip);
                tasks_status[name].y_solution = jac * solver_output_acc + bias_acc;
                tasks_status[name].y          = jac * robot_acc + bias_acc;
            }
        }
    }

    return tasks_status;
}

}
