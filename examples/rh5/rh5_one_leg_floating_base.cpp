#include <solvers/qpoases/QPOasesSolver.hpp>
#include <robot_models/RobotModelKDL.hpp>
#include <robot_models/RobotModelHyrodyn.hpp>
#include <types/QuadraticProgram.hpp>
#include <hyrodyn/robot_model_hyrodyn.hpp>
#include <Eigen/QR>
#include <scenes/AccelerationSceneTSID.hpp>
#include <Eigen/QR>

using namespace wbc;
using namespace std;
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

    base::samples::RigidBodyStateSE3 floating_base_state;
    floating_base_state.pose.position = base::Vector3d(-0.027769312129200783, 0.0, 0.918141273555804);
    floating_base_state.pose.orientation = base::Orientation(1,0,0,0);
    floating_base_state.twist.setZero();
    floating_base_state.acceleration.setZero();
    RobotModelConfig config("../../../models/urdf/rh5/rh5.urdf",
                           {"floating_base_trans_x", "floating_base_trans_y", "floating_base_trans_z",
                           "floating_base_rot_x",   "floating_base_rot_y",   "floating_base_rot_z",
                           "LLHip1", "LLHip2", "LLHip3", "LLKnee", "LLAnkleRoll", "LLAnklePitch"},
                           {"LLHip1", "LLHip2", "LLHip3", "LLKnee", "LLAnkleRoll", "LLAnklePitch"},
                           true,
                           "world",
                           floating_base_state,
                           {"LLAnkle_FT"});
    RobotModelPtr robot_model = std::make_shared<RobotModelKDL>();
    if(!robot_model->configure(config))
        return -1;

    std::vector<ConstraintConfig> wbc_config(1);
    wbc_config[0].name = "zero_com_acceleration";
    wbc_config[0].type = cart;
    wbc_config[0].root = "world";
    wbc_config[0].tip = "RH5_Root_Link";
    wbc_config[0].ref_frame = "world";
    wbc_config[0].priority = 0;
    wbc_config[0].weights = {1,1,1,1,1,1};
    wbc_config[0].activation = 1;

    QPSolverPtr solver = std::make_shared<QPOASESSolver>();
    Options options = std::dynamic_pointer_cast<QPOASESSolver>(solver)->getOptions();
    options.enableRegularisation = BT_TRUE;
    options.enableFarBounds = BT_FALSE;
    options.printLevel = PL_NONE;
    options.print();
    std::dynamic_pointer_cast<QPOASESSolver>(solver)->setOptions(options);
    options.print();
    std::dynamic_pointer_cast<QPOASESSolver>(solver)->setMaxNoWSR(1000);

    AccelerationSceneTSID scene(robot_model, solver);
    if(!scene.configure(wbc_config))

        return -1;
    uint nj = robot_model->noOfJoints();
    uint na = robot_model->noOfActuatedJoints();

    base::VectorXd q(na),qd(na),qdd(na);
    base::VectorXd solver_output;
    base::samples::Joints joint_state;

    hyrodyn::RobotModel_HyRoDyn robot_model_hyrodyn;
    robot_model_hyrodyn.load_robotmodel("../../../models/urdf/rh5/rh5_floating_base.urdf", "../../../models/hyrodyn/rh5/rh5_submechanisms_floating_base.yml");

    HierarchicalQP hqp;
    for(int n = 0; n < 1; n++){
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
        joint_state.names = robot_model->actuatedJointNames();
        for(size_t i = 0; i < na; i++){
            joint_state[i].position = q[i];
            joint_state[i].speed = qd[i];
            joint_state[i].acceleration = qdd[i];
        }
        joint_state.time = base::Time::now();
        robot_model->update(joint_state, floating_base_state);
        scene.getConstraint("zero_com_acceleration")->y_ref.setZero();
        hqp=scene.update();

        cout<<"A"<<endl;
        cout<<hqp[0].A<<endl;
        cout<<"H"<<endl;
        cout<<hqp[0].H<<endl;
        cout<<"g "<<hqp[0].g.transpose()<<endl;
        cout<<"lower_y "<<hqp[0].lower_y.transpose()<<endl;
        cout<<"upper_y "<<hqp[0].upper_y.transpose()<<endl;
        cout<<"lower_x "<<hqp[0].lower_x.transpose()<<endl;
        cout<<"upper_x "<<hqp[0].upper_x.transpose()<<endl;
        cout<<"Bias "<<robot_model->biasForces().transpose()<<endl<<endl;

        solver->solve(hqp,solver_output);

    }

    cout<<"Solution WBC: "<<endl;
    base::VectorXd acc_wbc = solver_output.segment(0,nj);
    base::VectorXd tau_wbc = solver_output.segment(nj,na);
    base::VectorXd f_ext_wbc = solver_output.segment(nj+na,6);
    cout<<"Acc:   "<<acc_wbc.transpose()<<endl;
    cout<<"Tau:   "<<tau_wbc.transpose()<<endl;
    cout<<"F_ext: "<<f_ext_wbc.transpose()<<endl<<endl;

    robot_model_hyrodyn.y.segment(0,3) = floating_base_state.pose.position;
    robot_model_hyrodyn.y.segment(3,3) = floating_base_state.pose.orientation.toRotationMatrix().eulerAngles(0, 1, 2);

    robot_model_hyrodyn.yd.setZero();
    robot_model_hyrodyn.ydd.setZero();
    robot_model_hyrodyn.ydd.segment(0,6) = acc_wbc.segment(0,6);
    for( unsigned int i = 6; i < robot_model_hyrodyn.jointnames_independent.size(); ++i ) {
        const std::string& name = robot_model_hyrodyn.jointnames_independent[i];
        robot_model_hyrodyn.y[i] = joint_state.getElementByName(name).position;
        robot_model_hyrodyn.yd[i] = joint_state.getElementByName(name).speed;
        robot_model_hyrodyn.ydd[i] = acc_wbc[i];
        robot_model_hyrodyn.Tau_independentjointspace[i] = joint_state.getElementByName(name).effort;
    }

    robot_model_hyrodyn.f_ext.resize(1);
    robot_model_hyrodyn.f_ext[0].set(f_ext_wbc(3),f_ext_wbc(4),f_ext_wbc(5),f_ext_wbc(0),f_ext_wbc(1),f_ext_wbc(2));
    robot_model_hyrodyn.wrench_points.push_back("LLAnkle_FT");
    robot_model_hyrodyn.wrench_resolution.push_back(true);  // Wrench is measure in Body Coordinates
    robot_model_hyrodyn.wrench_interaction.push_back(true); // Resistive

    robot_model_hyrodyn.calculate_inverse_dynamics();
    robot_model_hyrodyn.calculate_inverse_statics();

    cout<<"Solution Hyrodyn:"<<endl;
    cout<<"Tau actuated:                "<<robot_model_hyrodyn.Tau_actuated.transpose()<<endl;
    cout<<"Tau external:                "<<robot_model_hyrodyn.Tau_actuated_ext.transpose()<<endl;
    cout<<"Tau actuated + Tau external: "<<(robot_model_hyrodyn.Tau_actuated+robot_model_hyrodyn.Tau_actuated_ext).transpose()<<endl<<endl;

    return 0;
}
