#include <solvers/qpoases/QPOasesSolver.hpp>
#include <robot_models/RobotModelKDL.hpp>
#include <types/QuadraticProgram.hpp>
#include <hyrodyn/robot_model_hyrodyn.hpp>
#include <Eigen/QR>
#include <wbc/scenes/AccelerationScene.hpp>

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
    RobotModelConfig config("../../../models/urdf/rh5/rh5_one_leg.urdf",
                           {"LLHip1", "LLHip2", "LLHip3", "LLKnee", "LLAnkleRoll", "LLAnklePitch"},
                           {"LLHip1", "LLHip2", "LLHip3", "LLKnee", "LLAnkleRoll", "LLAnklePitch"});

    RobotModelPtr robot_model = std::make_shared<RobotModelKDL>();
    if(!robot_model->configure(config))
        return -1;

    QPSolverPtr solver = std::make_shared<QPOASESSolver>();
    Options options = std::dynamic_pointer_cast<QPOASESSolver>(solver)->getOptions();
    options.enableRegularisation = BT_TRUE;
    options.enableFarBounds = BT_FALSE;
    options.printLevel = PL_NONE;
    options.print();
    std::dynamic_pointer_cast<QPOASESSolver>(solver)->setOptions(options);
    std::dynamic_pointer_cast<QPOASESSolver>(solver)->setMaxNoWSR(1000);

    std::vector<ConstraintConfig> wbc_config(1);
    wbc_config[0].name = "left_leg_posture";
    wbc_config[0].type = cart;
    wbc_config[0].root = "RH5_Root_Link";
    wbc_config[0].tip = "LLAnkle_FT";
    wbc_config[0].ref_frame = "RH5_Root_Link";
    wbc_config[0].priority = 0;
    wbc_config[0].weights = {1,1,1,1,1,1};
    wbc_config[0].activation = 1;
    AccelerationScene scene(robot_model, solver);
    if(!scene.configure(wbc_config))
        return -1;

    uint nj = robot_model->noOfJoints();
    uint na = robot_model->noOfActuatedJoints();

    base::samples::Joints joint_state;
    base::VectorXd q,qd,qdd;
    q.resize(nj);
    qd.resize(nj);
    qdd.resize(nj);

    base::VectorXd solver_output;

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
        solver->solve(hqp, solver_output);

        std::cout<<"Solution Acc: "<<solver_output.transpose()<<endl;
    }

    return 0;
}
