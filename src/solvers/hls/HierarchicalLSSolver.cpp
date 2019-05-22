#include "HierarchicalLSSolver.hpp"
#include <stdexcept>
#include "SVD.hpp"
#include "../../types/QuadraticProgram.hpp"

using namespace std;

namespace wbc_solvers{

HierarchicalLSSolver::HierarchicalLSSolver() :
    no_of_joints(0),
    configured(false),
    min_eigenvalue(1e-9),
    max_solver_output_norm(10){
}

HierarchicalLSSolver::~HierarchicalLSSolver(){
}

bool HierarchicalLSSolver::configure(const std::vector<int>& n_constraints_per_prio, const unsigned int n_joints){

    priorities.clear();

    if(n_joints == 0)
        throw std::invalid_argument("Invalid Solver config. Number of joints must be > 0");

    if(n_constraints_per_prio.size() == 0)
        throw std::invalid_argument("Invalid Solver config. No of priority levels (size of n_constraints_per_prio) has to be > 0");

    for(uint i = 0; i < n_constraints_per_prio.size(); i++){
        if(n_constraints_per_prio[i] == 0)
            throw std::invalid_argument("Invalid Solver config. No of constraint variables on each priority level must be > 0");
    }

    for(uint prio = 0; prio < n_constraints_per_prio.size(); prio++)
        priorities.push_back(PriorityData(n_constraints_per_prio[prio], n_joints));

    no_of_joints = n_joints;
    proj_mat.resize(no_of_joints, no_of_joints);
    proj_mat.setIdentity();
    s_vals.setZero(no_of_joints);
    sing_vect_r.resize(no_of_joints, no_of_joints);
    sing_vect_r.setIdentity();
    s_vals_inv.setZero(no_of_joints, no_of_joints);
    damped_s_vals_inv.setZero(no_of_joints, no_of_joints);
    Wq_V.setZero(no_of_joints, no_of_joints);
    Wq_V_s_vals_inv.setZero(no_of_joints, no_of_joints);
    Wq_V_damped_s_vals_inv.setZero(no_of_joints, no_of_joints);
    tmp.setZero(no_of_joints);

    configured = true;
    return true;
}

void HierarchicalLSSolver::solve(const wbc::HierarchicalQP &hierarchical_qp, base::VectorXd &solver_output){

    if(!configured){
        uint n_joints;
        std::vector<int> n_constraints_per_prio;
        for(size_t i = 0; i < hierarchical_qp.size(); i++){
            n_constraints_per_prio.push_back(hierarchical_qp[i].A.rows());
            n_joints = hierarchical_qp[i].A.cols();
        }
        if(!configure(n_constraints_per_prio, n_joints))
            throw std::runtime_error("Solver has not been configured yet!");
    }

    // Check valid input
    if(hierarchical_qp.size() != priorities.size())
        throw std::invalid_argument("Invalid solver input. Number of priorities in solver: " + std::to_string(priorities.size())
                                    + ", Size of input vector: " + std::to_string(hierarchical_qp.size()));

    solver_output.setZero(no_of_joints);

    // Init projection matrix as identity, so that the highest priority can look for a solution in whole configuration space
    proj_mat.setIdentity();

    //////// Loop through all priorities

    for(uint prio = 0; prio < priorities.size(); prio++){

        if(hierarchical_qp[prio].A.rows()        != priorities[prio].n_constraint_variables ||
           hierarchical_qp[prio].A.cols()        != no_of_joints ||
           hierarchical_qp[prio].lower_y.size()  != priorities[prio].n_constraint_variables){

            string nc = to_string(priorities[prio].n_constraint_variables), nq = to_string(no_of_joints);
            string a_rows = to_string(hierarchical_qp[prio].A.rows()), a_cols = to_string(hierarchical_qp[prio].A.cols());
            string y_rows = to_string(hierarchical_qp[prio].lower_y.size());
            throw std::invalid_argument("Expected input size on priority level " + to_string(prio) + ": " +  "A: " + nc + " x " + nq +
                      ", b: " + nc + " x 1, actual input: " + "A: " + a_rows + " x " + a_cols +", b: " + y_rows + " x 1");
        }

        // Set weights for this prioritiy
        setConstraintWeights(hierarchical_qp[prio].Wy, prio);
        setJointWeights(hierarchical_qp.Wq, prio);

        priorities[prio].y_comp.setZero();

        // Compensate y for part of the solution already met in higher priorities. For the first priority y_comp will be equal to  y
        priorities[prio].y_comp = hierarchical_qp[prio].lower_y - hierarchical_qp[prio].A*solver_output;

        // projection of A on the null space of previous priorities: A_proj = A * P = A * ( P(p-1) - (A_wdls)^# * A )
        // For the first priority P == Identity
        priorities[prio].A_proj = hierarchical_qp[prio].A * proj_mat;

        // Compute weighted, projected mat: A_proj_w = Wy * A_proj * Wq^-1
        // Since the weight matrices are diagonal, there is no need for full matrix multiplication

        for(uint i = 0; i < priorities[prio].n_constraint_variables; i++)
            priorities[prio].A_proj_w.row(i) = priorities[prio].constraint_weight_mat(i,i) * priorities[prio].A_proj.row(i);

        for(uint i = 0; i < no_of_joints; i++)
            priorities[prio].A_proj_w.col(i) = priorities[prio].joint_weight_mat(i,i) * priorities[prio].A_proj_w.col(i);

        svd_eigen_decomposition(priorities[prio].A_proj_w, priorities[prio].U, s_vals, sing_vect_r, tmp);

        // Compute damping factor based on
        // A.A. Maciejewski, C.A. Klein, “Numerical Filtering for the Operation of
        // Robotic Manipulators through Kinematically Singular Configurations”,
        // Journal of Robotic Systems, Vol. 5, No. 6, pp. 527 - 552, 1988.
        double s_min = s_vals.block(0,0,min(no_of_joints, priorities[prio].n_constraint_variables),1).minCoeff();
        if(s_min <= (1/max_solver_output_norm)/2)
            priorities[prio].damping = (1/max_solver_output_norm)/2;
        else if(s_min >= (1/max_solver_output_norm))
            priorities[prio].damping = 0;
        else
            priorities[prio].damping = sqrt(s_min*((1/max_solver_output_norm)-s_min));

        // Damped Inverse of Eigenvalue matrix for computation of a singularity robust solution for the current priority
        damped_s_vals_inv.setZero();
        for (uint i = 0; i < min(no_of_joints, priorities[prio].n_constraint_variables); i++)
            damped_s_vals_inv(i,i) = (s_vals(i) / (s_vals(i) * s_vals(i) + priorities[prio].damping * priorities[prio].damping));

        // Additionally compute normal Inverse of Eigenvalue matrix for correct computation of nullspace projection
        for(uint i = 0; i < s_vals.rows(); i++){
            if(s_vals(i) < min_eigenvalue)
                s_vals_inv(i,i) = 0;
            else
                s_vals_inv(i,i) = 1 / s_vals(i);
        }

        // A^# = Wq^-1 * V * S^# * U^T * Wy
        // Since the weight matrices are diagonal, there is no need for full matrix multiplication (saves a lot of computation!)
        priorities[prio].u_t_weight_mat = priorities[prio].U.transpose();
        for(uint i = 0; i < priorities[prio].n_constraint_variables; i++)
            priorities[prio].u_t_weight_mat.col(i) = priorities[prio].u_t_weight_mat.col(i) * priorities[prio].constraint_weight_mat(i,i);

        for(uint i = 0; i < no_of_joints; i++)
            Wq_V.row(i) = priorities[prio].joint_weight_mat(i,i) * sing_vect_r.row(i);

        for(uint i = 0; i < no_of_joints; i++)
            Wq_V_s_vals_inv.col(i) = Wq_V.col(i) * s_vals_inv(i,i);
        for(uint i = 0; i < no_of_joints; i++)
            Wq_V_damped_s_vals_inv.col(i) = Wq_V.col(i) * damped_s_vals_inv(i,i);

        priorities[prio].A_proj_inv_wls = Wq_V_s_vals_inv * priorities[prio].u_t_weight_mat; //Normal Inverse with weighting
        priorities[prio].A_proj_inv_wdls = Wq_V_damped_s_vals_inv * priorities[prio].u_t_weight_mat; //Damped inverse with weighting

        // x = x + A^# * y
        priorities[prio].solution_prio = priorities[prio].A_proj_inv_wdls * priorities[prio].y_comp;
        solver_output += priorities[prio].solution_prio;

        // Compute projection matrix for the next priority. Use here the undamped inverse to have a correct solution
        proj_mat -= priorities[prio].A_proj_inv_wls * priorities[prio].A_proj;

        //store eigenvalues for this priority
        priorities[prio].sing_vals.setZero();
        for(uint i = 0; i < no_of_joints; i++)
            priorities[prio].sing_vals(i) = s_vals(i);

    } //priority loop

    ///////////////
}

void HierarchicalLSSolver::setJointWeights(const base::VectorXd& weights){
    if(!configured)
        throw std::runtime_error("setJointWeights: Solver has not been configured yet!");
    for(size_t i = 0; i < priorities.size(); i++)
        setJointWeights(weights, i);
}

void HierarchicalLSSolver::setJointWeights(const base::VectorXd& weights, const uint prio){
    if(!configured)
        throw std::runtime_error("setJointWeights: Solver has not been configured yet!");
    if(prio < 0 ||prio >= priorities.size())
        throw std::invalid_argument("Cannot set joint weights on priority " + to_string(prio) +
                                    ". Number of priority levels is " + to_string(priorities.size()));

    if(weights.size() == no_of_joints){
        priorities[prio].joint_weight_mat.setZero();
        for(uint i = 0; i < no_of_joints; i++)
        {
            if(weights(i) >= 0)
                priorities[prio].joint_weight_mat(i,i) = sqrt(weights(i));
            else
                throw std::invalid_argument("Entries of joint weight vector have to be >= 0, but element " + to_string(i) + " is " + to_string(weights(i)));
        }
    }
    else{
        throw std::invalid_argument("Cannot set joint weights. Size of joint weight vector is " + to_string(weights.size()) + " but should be " + to_string(no_of_joints));
    }
}

void HierarchicalLSSolver::setConstraintWeights(const base::VectorXd& weights, const uint prio){
    if(!configured)
        throw std::runtime_error("setConstraintWeights: Solver has not been configured yet!");

    if(prio < 0 ||prio >= priorities.size())
        throw std::invalid_argument("Cannot set constraint weights on priority " + to_string(prio) +
                                    ". Number of priority levels is " + to_string(priorities.size()));

    if(priorities[prio].n_constraint_variables != weights.size())
        throw std::invalid_argument("Cannot set joint weights. Size of joint weight vector is " + to_string(weights.size())
                                    + " but should be " + to_string(priorities[prio].n_constraint_variables));
    priorities[prio].constraint_weight_mat.setZero();
    for(uint i = 0; i < priorities[prio].n_constraint_variables; i++){
        if(weights(i) >= 0)
           priorities[prio].constraint_weight_mat(i,i) = sqrt(weights(i));
        else
            throw std::invalid_argument("Entries of constraint weight vector have to be >= 0, but element " + to_string(i) + " is " + to_string(weights(i)));

    }
}

void HierarchicalLSSolver::setMinEigenvalue(double _min_eigenvalue){
    if(_min_eigenvalue <= 0){
        throw std::invalid_argument("Min. Eigenvalue has to be > 0!");
    }
    min_eigenvalue = _min_eigenvalue;
}

void HierarchicalLSSolver::setMaxSolverOutputNorm(double norm_max){
    if(norm_max <= 0){
        throw std::invalid_argument("Norm Max has to be > 0!");
    }
    max_solver_output_norm = norm_max;
}
}
