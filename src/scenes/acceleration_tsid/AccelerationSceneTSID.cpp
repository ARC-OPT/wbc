#include "AccelerationSceneTSID.hpp"
#include "core/RobotModel.hpp"
#include "../../tools/Logger.hpp"

#include "../../constraints/RigidbodyDynamicsConstraint.hpp"
#include "../../constraints/ContactsAccelerationConstraint.hpp"
#include "../../constraints/JointLimitsAccelerationConstraint.hpp"
#include "../../constraints/ContactsFrictionPointConstraint.hpp"

namespace wbc {

SceneRegistry<AccelerationSceneTSID> AccelerationSceneTSID::reg("acceleration_tsid");

AccelerationSceneTSID::AccelerationSceneTSID(RobotModelPtr robot_model, QPSolverPtr solver, const double dt) :
    Scene(robot_model, solver, dt),
    hessian_regularizer(1e-8),
    configured(false){

    // whether or not torques are removed  from the qp problem
    // this formulation includes torques !!!
    bool reduced = false; // DO NOT CHANGE

    // for now manually adding constraint to this scene (an option would be to take them during configuration)
    constraints.push_back(std::make_shared<RigidbodyDynamicsConstraint>(reduced));
    constraints.push_back(std::make_shared<ContactsAccelerationConstraint>(reduced));
    constraints.push_back(std::make_shared<JointLimitsAccelerationConstraint>(dt, reduced));
    constraints.push_back(std::make_shared<ContactsFrictionPointConstraint>(reduced));
}

bool AccelerationSceneTSID::configure(const std::vector<TaskPtr> &tasks_in){
    if(tasks_in.empty()){
        log(logERROR)<<"Empty WBC Task configuration";
        return false;
    }
    for(const TaskPtr& task : tasks_in){
        if(task->type != spatial_acceleration &&
           task->type != joint_acceleration &&
           task->type != com_acceleration &&
           task->type != contact_force){
            log(logERROR)<<"Invalid task type: "<<task->type;
            return false;
        }
    }

    solver->reset();
    tasks.resize(1);
    tasks = tasks_in;
    hqp.resize(1);
    configured = true;

    return true;
}

const HierarchicalQP& AccelerationSceneTSID::update(){

    assert(configured);

    uint nj = robot_model->nj();
    uint na = robot_model->na();
    uint ncp = robot_model->nc();

    ///////// Constraints

    bool has_bounds = false;
    size_t total_eqs = 0, total_ineqs = 0;
    for(auto contraint : constraints) {
        contraint->update(robot_model);
        if(contraint->type() == Constraint::equality)
            total_eqs += contraint->size();
        if(contraint->type() == Constraint::inequality)
            total_ineqs += contraint->size();
        if(contraint->type() == Constraint::bounds)
            has_bounds = true;;
    }

    // QP Size: (nc x nj+na+nc*3)
    //Variable order: (qdd,tau,f_ext)
    QuadraticProgram& qp = hqp[0];
    qp.resize(nj+na+ncp*3, total_eqs, total_ineqs, has_bounds);
    total_eqs = total_ineqs = 0;
    for(uint i = 0; i < constraints.size(); i++) {
        Constraint::Type type = constraints[i]->type();
        size_t c_size = constraints[i]->size();

        if(type == Constraint::bounds) {
            qp.lower_x = constraints[i]->lb();
            qp.upper_x = constraints[i]->ub();
        }
        else if (type == Constraint::equality) {
            qp.A.middleRows(total_eqs, c_size) = constraints[i]->A();
            qp.b.segment(total_eqs, c_size) = constraints[i]->b();
            total_eqs += c_size;
        }
        else if (type == Constraint::inequality) {
            qp.C.middleRows(total_ineqs, c_size) = constraints[i]->A();
            qp.lower_y.segment(total_ineqs, c_size) = constraints[i]->lb();
            qp.upper_y.segment(total_ineqs, c_size) = constraints[i]->ub();
            total_ineqs += c_size;
        }
    }

    ///////// Tasks

    qp.H.setZero();
    qp.g.setZero();
    uint wrench_idx = 0;
    hqp.Wq = Eigen::VectorXd::Map(robot_model->getJointWeights().data(), robot_model->nj());
    for(uint i = 0; i < tasks.size(); i++){
        
        TaskPtr task = tasks[i];
        task->update();

        // If the activation value is zero, also set reference to zero. Activation is usually used to switch between different
        // task phases and we don't want to store the "old" reference value, in case we switch on the task again
        if(task->activation == 0){
           task->y_ref.setZero();
           task->y_ref_world.setZero();
        }

        for(int i = 0; i < task->A.rows(); i++){
            task->Aw.row(i) = task->weights_world(i) * task->A.row(i) * task->activation;
            task->y_ref_world(i) = task->y_ref_world(i) * task->weights_world(i) * task->activation;
        }
        for(int i = 0; i < task->A.cols(); i++)
            task->Aw.col(i) = hqp.Wq[i] * task->Aw.col(i);

        // Decide on which output variables the tasks are to be mapped: qdd or f_ext
        if(task->type == TaskType::contact_force){ // f_ext
            /*const std::vector<ActiveContact> &cp = robot_model->getActiveContacts();
            const std::string contact_link = task->config.ref_frame;
            if(std::find(cp.names.begin(), cp.names.end(), contact_link) == cp.names.end()){
                LOG_ERROR("Reference frame defined in wrench task %s must match one of the contact points defined robot model", task->config.name.c_str());
                LOG_ERROR("You set reference frame %s, but this frame is not a contact point", contact_link.c_str());
                throw std::runtime_error("Invalid task configuration");
            }*/

            qp.H.block(nj+na+wrench_idx*3,nj+na+wrench_idx*3,3,3) += task->Aw.transpose()*task->Aw;
            qp.g.segment(nj+na+wrench_idx*3,3) -= task->Aw.transpose()*task->y_ref_world;
            wrench_idx++;
        }
        else{ // qdd
            qp.H.block(0,0,nj,nj) += task->Aw.transpose()*task->Aw;
            qp.g.segment(0,nj) -= task->Aw.transpose()*task->y_ref_world;
        }
    }

    qp.H.diagonal().array() += hessian_regularizer;

    return hqp;
}

const types::JointCommand& AccelerationSceneTSID::solve(const HierarchicalQP& hqp){

    // solve
    solver_output.resize(hqp[0].nq);
    bool allow_warm_start = !contactsHaveChanged(contacts, robot_model->getContacts());
    contacts = robot_model->getContacts();
    solver->solve(hqp, solver_output, allow_warm_start);

    // Convert solver output: Acceleration and torque
    uint nj = robot_model->nj();
    uint na = robot_model->na();
    uint nfb = robot_model->nfb();
    solver_output_joints.resize(na);
    for(uint i = 0; i < solver_output.size(); i++){
        if(std::isnan(solver_output[i]))
            throw std::runtime_error("Solver output is NaN");
    }
    solver_output_joints.acceleration = solver_output.segment(nfb,nj-nfb);
    solver_output_joints.effort = solver_output.segment(nj,na);

    // std::cout<<"Acc:   "<<solver_output.segment(0,nj).transpose()<<std::endl;
    // std::cout<<"Tau:   "<<solver_output.segment(nj,na).transpose()<<std::endl;
    // std::cout<<"F_ext: "<<solver_output.segment(nj+na,robot_model->nc()*3).transpose()<<std::endl<<std::endl;

    // Convert solver output: contact wrenches
    contact_wrenches.resize(robot_model->getContacts().size());
    for(uint i = 0; i < robot_model->getContacts().size(); i++){
        contact_wrenches[i].force = solver_output.segment(nj+na+i*3,3);
        //contact_wrenches[i].torque = solver_output.segment(nj+na+i*6+3,3);
    }
    return solver_output_joints;
}

}
