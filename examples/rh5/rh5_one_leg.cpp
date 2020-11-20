#include <solvers/qpoases/QPOasesSolver.hpp>
#include <robot_models/RobotModelKDL.hpp>
#include <types/QuadraticProgram.hpp>
#include <hyrodyn/robot_model_hyrodyn.hpp>
#include <Eigen/QR>
#include <wbc/scenes/AccelerationScene.hpp>

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
    vector<RobotModelConfig> configs;
    configs.push_back(RobotModelConfig("../../../models/urdf/rh5/rh5.urdf",
                                       {"LLHip1", "LLHip2", "LLHip3", "LLKnee", "LLAnkleRoll", "LLAnklePitch"},
                                       {"LLHip1", "LLHip2", "LLHip3", "LLKnee", "LLAnkleRoll", "LLAnklePitch"},
                                       wbc::ModelType::URDF));

    RobotModelPtr robot_model = std::make_shared<RobotModelKDL>();
    if(!robot_model->configure(configs))
        return -1;

    ConstraintConfig config;
    config.name = "left_leg_posture";
    config.type = cart;
    config.root = "RH5_Root_Link";
    config.tip = "LLAnkle_FT";
    config.ref_frame = "RH5_Root_Link";
    config.priority = 0;
    config.weights = {1,1,1,1,1,1};
    config.activation = 1;
    std::vector<ConstraintConfig> constraint_configs;
    constraint_configs.push_back(config);
    WbcAccelerationScene scene(robot_model);
    if(!scene.configure(constraint_configs))
        return -1;

    QPOASESSolver solver;
    Options options = solver.getOptions();
    options.enableRegularisation = BT_TRUE;
    options.enableFarBounds = BT_FALSE;
    options.printLevel = PL_NONE;
    options.print();
    solver.setOptions(options);
    solver.setMaxNoWSR(1000);

    uint nj = robot_model->noOfJoints();
    uint na = robot_model->noOfActuatedJoints();

    base::VectorXd solver_output;
    base::samples::Joints joint_state;
    base::VectorXd q,qd,qdd;
    q.resize(nj);
    qd.resize(nj);
    qdd.resize(nj);

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
        joint_state.names = robot_model->actuatedJointNames();
        for(size_t i = 0; i < na; i++){
            joint_state[i].position = q[i];
            joint_state[i].speed = qd[i];
            joint_state[i].acceleration = qdd[i];
        }
        joint_state.time = base::Time::now();
        robot_model->update(joint_state);

        for(int i = 0; i < 6; i++)
            scene.getConstraint("left_leg_posture")->y_ref[i] = whiteNoise(1e-4);

        scene.update();
        HierarchicalQP hqp;
        scene.getHierarchicalQP(hqp);

        solver.solve(hqp, solver_output);

        std::cout<<"Solution Acc: "<<solver_output.transpose()<<std::endl;
    }

    return 0;
}
