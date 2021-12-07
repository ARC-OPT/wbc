#include <robot_models/hyrodyn/RobotModelHyrodyn.hpp>
#include <solvers/qpoases/QPOasesSolver.hpp>
#include <scenes/AccelerationSceneTSID.hpp>
#include <tools/URDFTools.hpp>

using namespace std;
using namespace wbc;

int main()
{
    RobotModelPtr robot_model = make_shared<RobotModelHyrodyn>();
    RobotModelConfig config("../../../models/rh5v2/urdf/rh5v2.urdf");
    config.joint_blacklist = {"HeadPitch", "HeadRoll", "HeadYaw",
                              "GLF1Gear", "GLF1ProximalSegment", "GLF1TopSegment",
                              "GLF2Gear", "GLF2ProximalSegment", "GLF2TopSegment",
                              "GLF3Gear", "GLF3ProximalSegment", "GLF3TopSegment",
                              "GLF4Gear", "GLF4ProximalSegment", "GLF4TopSegment", "GLThumb",
                              "GRF1Gear", "GRF1ProximalSegment", "GRF1TopSegment",
                              "GRF2Gear", "GRF2ProximalSegment", "GRF2TopSegment"};
    config.submechanism_file = "../../../models/rh5v2/hyrodyn/rh5v2.yml";
    if(!robot_model->configure(config))
        return -1;

    base::samples::Joints joint_state;
    joint_state.names = robot_model->jointNames();
    for(auto n : joint_state.names){
        base::JointState js;
        js.position = js.speed = js.acceleration = 0;
        joint_state.elements.push_back(js);
    }

    joint_state.time = base::Time::now();

    joint_state["ALShoulder1"].position = joint_state["ARShoulder1"].position = -1.0;
    joint_state["ALShoulder2"].position = joint_state["ARShoulder2"].position = 1.0;
    joint_state["ALElbow"].position     = joint_state["ARElbow"].position     = -1.0;
    robot_model->update(joint_state);

    QPSolverPtr solver = make_shared<QPOASESSolver>();
    dynamic_pointer_cast<QPOASESSolver>(solver)->setMaxNoWSR(100);
    qpOASES::Options options = dynamic_pointer_cast<QPOASESSolver>(solver)->getOptions();
    options.printLevel = qpOASES::PL_NONE;
    dynamic_pointer_cast<QPOASESSolver>(solver)->setOptions(options);

    AccelerationSceneTSID scene(robot_model, solver);
    vector<ConstraintConfig> wbc_config;
    wbc_config.push_back(ConstraintConfig("cart_ctrl_left",  0, "RH5v2_Root_Link", "ALWristFT_Link", "RH5v2_Root_Link", 1.0));
    wbc_config.push_back(ConstraintConfig("cart_ctrl_right",  0, "RH5v2_Root_Link", "ARWristFT_Link", "RH5v2_Root_Link", 1.0));
    if(!scene.configure(wbc_config))
        return -1;
    HierarchicalQP hqp = scene.update();
    base::commands::Joints solver_output = scene.solve(hqp);
    std::cout<<"Solver Output"<<std::endl;
    for(auto n : solver_output.names)
        cout<<n<<": "<<solver_output[n].acceleration<<" "<<solver_output[n].effort<<endl;
    std::cout<<"Bias forces"<<std::endl;
    cout<<robot_model->biasForces().transpose()<<endl;
    std::cout<<"..............................................."<<std::endl;

    return 0;
}
