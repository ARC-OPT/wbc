#include <hyrodyn/robot_model_hyrodyn.hpp>
#include <chrono>

using namespace std;

int main(){

    hyrodyn::RobotModel_HyRoDyn hyrodyn;
    string urdf_file = "../../../models/hyper-2/submechanisms/HyPer-2_full.urdf";
    string submechanism_file = "../../../models/hyper-2/submechanisms/submechanisms.yml";
    hyrodyn.load_robotmodel(urdf_file, submechanism_file);

    hyrodyn.y   = Eigen::VectorXd::Zero(hyrodyn.active_dof);
    hyrodyn.yd  = Eigen::VectorXd::Zero(hyrodyn.active_dof);
    hyrodyn.ydd = Eigen::VectorXd::Zero(hyrodyn.active_dof);
    hyrodyn.Tau_independentjointspace_input = Eigen::VectorXd::Zero(hyrodyn.active_dof);

    hyrodyn.y << 0.0,  0.0,-0.3, 0.6,-0.33, 0.0, // Left Leg
                 0.0,  0.0,-0.3, 0.6,-0.33, 0.0, // Right Leg
                 0.0,-1.57, 0.0,-0.7,            // Left Arm
                 0.0, 1.57, 0.0, 0.7;            // Right Arm
    chrono::steady_clock::time_point begin = chrono::steady_clock::now();
    hyrodyn.calculate_system_state();
    hyrodyn.calculate_simplified_inverse_dynamics();
    chrono::steady_clock::time_point end = chrono::steady_clock::now();
    cout << "Time difference = " << chrono::duration_cast<chrono::microseconds>(end - begin).count()/1000.0 << "[ms]" << endl;

    cout<<"Active Joints: ";
    for(auto n : hyrodyn.jointnames_active) cout<<n<<" "; cout<<endl;
    cout<<"Independent Joints: ";
    for(auto n : hyrodyn.jointnames_active) cout<<n<<" "; cout<<endl;
    cout<<hyrodyn.u.transpose()<<endl;
    cout<<hyrodyn.ud.transpose()<<endl;
    cout<<hyrodyn.udd.transpose()<<endl;
    cout<<hyrodyn.Tau_actuated.transpose()<<endl;

    return 0;
}
