#include <solvers/qpoases/QPOasesSolver.hpp>
#include <robot_models/RobotModelKDL.hpp>
#include <types/QuadraticProgram.hpp>
#include <hyrodyn/robot_model_hyrodyn.hpp>
#include <Eigen/QR>

using namespace wbc;
using namespace std;
using namespace wbc_solvers;
using namespace qpOASES;

double whiteNoise(const double std_dev)
{
    double rand_no = ( rand() / ( (double)RAND_MAX ) );
    while( rand_no == 0 )
        rand_no = ( rand() / ( (double)RAND_MAX ) );

    double tmp = cos( ( 2.0 * (double)M_PI ) * rand() / ( (double)RAND_MAX ) );
    return std_dev * sqrt( -2.0 * log( rand_no ) ) * tmp;
}


int main(){

    const string world_link = "world";
    const string contact_link = "LLAnkle_FT";

    base::samples::RigidBodyStateSE3 floating_base_state;
    floating_base_state.pose.position = base::Vector3d(-0.027769312129200783, 0.0, 0.918141273555804);
    floating_base_state.pose.orientation = base::Orientation(1,0,0,0);
    floating_base_state.twist.setZero();
    floating_base_state.acceleration.setZero();
    vector<RobotModelConfig> configs;
    configs.push_back(RobotModelConfig("../../../models/urdf/rh5/rh5.urdf",
                                       {"floating_base_trans_x", "floating_base_trans_y", "floating_base_trans_z",
                                        "floating_base_rot_x",   "floating_base_rot_y",   "floating_base_rot_z",
                                        "LLHip1", "LLHip2", "LLHip3", "LLKnee", "LLAnkleRoll", "LLAnklePitch"},
                                       {"LLHip1", "LLHip2", "LLHip3", "LLKnee", "LLAnkleRoll", "LLAnklePitch"},
                                       wbc::ModelType::URDF,
                                       true,
                                       "world",
                                       floating_base_state));
    RobotModelKDL robot_model;
    if(!robot_model.configure(configs))
        return -1;

    QPOASESSolver solver;
    Options options = solver.getOptions();
    options.enableRegularisation = BT_TRUE;
    options.enableFarBounds = BT_FALSE;
    options.printLevel = PL_NONE;
    options.print();
    solver.setOptions(options);
    options.print();
    solver.setMaxNoWSR(1000);

    uint nj = robot_model.noOfJoints();
    uint na = robot_model.noOfActuatedJoints();

    QuadraticProgram qp;
    qp.resize(nj,nj+na); // (nc,nq)

    base::VectorXd q(na),qd(na),qdd(na);
    base::VectorXd solver_output;
    base::samples::Joints joint_state;

    base::Vector6d f_ext;
    f_ext << -0.0016937263217112032, -0.0031835690480501865, -75.94681724308107, -0.07960479827699948, 2.7142051814597408, -0.00011199983616109183;

    hyrodyn::RobotModel_HyRoDyn robot_model_hyrodyn;
    robot_model_hyrodyn.load_robotmodel("../../../models/urdf/rh5/rh5_floating_base.urdf", "../../../models/hyrodyn/rh5/rh5_submechanisms_floating_base.yml");


    Eigen::MatrixXd pinv;

    for(int n = 0; n < 10; n++){
        cout<<"------------------- Iteration "<<n<<" ---------------------"<<endl;

        q << 0,0,-0.2,0.4,-0.2,0;
        qd.setZero(na);
        qdd.setZero(na);
        for(int i = 0; i < 6; i++){
            q(i) += whiteNoise(1e-4);
            qd(i) += whiteNoise(1e-4);
            qdd(i) += whiteNoise(1e-4);
        }

        joint_state.resize(na);
        joint_state.names = robot_model.actuatedJointNames();
        for(size_t i = 0; i < na; i++){
            joint_state[i].position = q[i];
            joint_state[i].speed = qd[i];
            joint_state[i].acceleration = qdd[i];
        }
        joint_state.time = base::Time::now();
        robot_model.update(joint_state, floating_base_state);
        base::VectorXd vel(nj);
        vel.setZero();
        vel.segment(6,6) = qd;

        // Constraint Matrix
        base::MatrixXd A = robot_model.spaceJacobian(world_link, contact_link);
        // Target Acceleration of contact point
        base::Vector6d y = -robot_model.jacobianDot(world_link, contact_link) * vel;

        // Cost function

        // Only minimize acceleration, not torques
        qp.H.setZero();
        qp.H.block(0,0,nj,nj) = A.transpose()*A;
        qp.g.setZero();
        qp.g.segment(0,nj) = -(A.transpose()*y).transpose();

        // Constraints

        // [[M -S^T]] * [qdd tau] = [-h + J^T * f_ext]

        qp.A.setZero();
        //qp.A.block(0,  0, 6, nj) =  A;
        qp.A.block(0,  0, nj, nj) =  robot_model.jointSpaceInertiaMatrix();
        qp.A.block(0, nj, nj, na) = -robot_model.selectionMatrix().transpose();
        qp.lower_y.segment(0,nj) = qp.upper_y.segment(0,nj) = -robot_model.biasForces() + A.transpose() * f_ext;
        qp.lower_y.segment(0,6).setZero();
        qp.upper_y.segment(0,6).setZero();
        qp.lower_x.setConstant(-10000);
        qp.upper_x.setConstant(10000);

        pinv = qp.A.completeOrthogonalDecomposition().pseudoInverse();

        cout<<"Bias: "<<robot_model.biasForces().transpose()<<endl;
        /*cout<<"H"<<endl;
        cout<<qp.H<<endl;
        cout<<"g"<<endl;
        cout<<qp.g.transpose()<<endl;*/
        cout<<"A.transpose() * f_ext"<<endl;
        cout<<A.transpose() * f_ext<<endl;
        cout<<"A"<<endl;
        cout<<qp.A<<endl;
        cout<<"lower_y"<<endl;
        cout<<qp.lower_y.transpose()<<endl;
        cout<<"upper_y"<<endl;
        cout<<qp.upper_y.transpose()<<endl;
        /*cout<<"lower_x"<<endl;
        cout<<qp.lower_x.transpose()<<endl;
        cout<<"upper_x"<<endl;
        cout<<qp.upper_x.transpose()<<endl<<endl;
        cout<<"M"<<endl;
        cout<<robot_model.jointSpaceInertiaMatrix()<<endl<<endl;
        cout<<"J"<<endl;
        cout<<A<<endl<<endl;*/

        HierarchicalQP hqp;
        hqp.actuated_joint_names = robot_model.actuatedJointNames();
        hqp.joint_names = robot_model.jointNames();
        hqp.joint_state = robot_model.jointState(robot_model.jointNames());
        hqp.Wq.setOnes(na);
        hqp.time = base::Time::now();
        hqp << qp;

        // Solve

        solver.solve(hqp,solver_output);
    }

    robot_model_hyrodyn.y.segment(0,3) = floating_base_state.pose.position;
    robot_model_hyrodyn.y.segment(3,3) = floating_base_state.pose.orientation.toRotationMatrix().eulerAngles(0, 1, 2);

    robot_model_hyrodyn.yd.setZero();
    robot_model_hyrodyn.ydd.setZero();

    for( unsigned int i = 6; i < robot_model_hyrodyn.jointnames_independent.size(); ++i ) {
        const std::string& name = robot_model_hyrodyn.jointnames_independent[i];
        std::size_t idx = joint_state.mapNameToIndex(name);
        robot_model_hyrodyn.y[i] = joint_state.getElementByName(name).position;
        robot_model_hyrodyn.yd[i] = joint_state.getElementByName(name).speed;
        robot_model_hyrodyn.ydd[i] = joint_state.getElementByName(name).acceleration;
        robot_model_hyrodyn.Tau_independentjointspace[i] = joint_state.getElementByName(name).effort;
    }

    robot_model_hyrodyn.f_ext.resize(1);
    robot_model_hyrodyn.f_ext[0].set(f_ext(3),f_ext(4),f_ext(5),f_ext(0),f_ext(1),f_ext(2));
    robot_model_hyrodyn.wrench_points.push_back("LLAnkle_FT");
    robot_model_hyrodyn.wrench_resolution.push_back(false);
    robot_model_hyrodyn.wrench_interaction.push_back(true);

    robot_model_hyrodyn.calculate_inverse_dynamics();
    robot_model_hyrodyn.calculate_inverse_statics();
    robot_model_hyrodyn.calculate_constrained_inverse_dynamics();


    cout<<endl;
    cout<<"Solution WBC: "<<endl;
    cout<<"Acc:   "<<solver_output.segment(0,nj).transpose()<<endl;
    cout<<"Tau:   "<<solver_output.segment(nj,na).transpose()<<endl;
    cout<<"F_ext: "<<solver_output.segment(nj+na,6).transpose()<<endl;
    cout<<endl;
    cout<<"Solution Hyrodyn:"<<endl;
    cout<<"Tau actuated:                "<<robot_model_hyrodyn.Tau_actuated.transpose()<<endl;
    cout<<"Tau external:                "<<robot_model_hyrodyn.Tau_actuated_ext.transpose()<<endl;
    cout<<"Tau actuated + Tau external: "<<(robot_model_hyrodyn.Tau_actuated+robot_model_hyrodyn.Tau_actuated_ext).transpose()<<endl;
    cout<<"Solution pinv: "<<endl;
    cout<<"Acc:   "<<(pinv*qp.lower_y).segment(0,nj).transpose()<<endl;
    cout<<"tau:   "<<(pinv*qp.lower_y).segment(nj,na).transpose()<<endl;
    cout<<"F_ext: "<<(pinv*qp.lower_y).segment(nj+na,6).transpose()<<endl;

    /*cout<<"Rigid Body Dynamics check: "<<endl;
    base::VectorXd bias_check = robot_model.jointSpaceInertiaMatrix()*solver_output.segment(0,nj) +
                                -robot_model.jacobian(world_link,contact_link).transpose() * solver_output.segment(nj,6) +
                                -robot_model.selectionMatrix().transpose()*solver_output.segment(nj+6,na);
    cout<<bias_check.transpose()<<endl;
    cout<<robot_model.biasForces().transpose()<<endl;*/

    return 0;
}
