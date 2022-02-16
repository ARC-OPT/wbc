#include "benchmarks_common.hpp"
#include <fstream>

using namespace std;

namespace wbc{

double stdDev(const base::VectorXd& vec){
    return sqrt((vec.array() - vec.mean()).square().sum() / (vec.size() - 1));
}

double whiteNoise(const double std_dev)
{
    double rand_no = ( rand() / ( (double)RAND_MAX ) );
    while( rand_no == 0 )
        rand_no = ( rand() / ( (double)RAND_MAX ) );

    double tmp = cos( ( 2.0 * (double)M_PI ) * rand() / ( (double)RAND_MAX ) );
    return std_dev * sqrt( -2.0 * log( rand_no ) ) * tmp;
}

base::samples::Joints randomJointState(const vector<string>& joint_names, const base::JointLimits& limits){
    base::samples::Joints joint_state;
    joint_state.resize(joint_names.size());
    joint_state.names = joint_names;
    for(auto n : joint_state.names){
        joint_state[n].position = (double(rand())/RAND_MAX)* (limits[n].max.position - limits[n].min.position) + limits[n].min.position;
        joint_state[n].speed = whiteNoise(1e-4);
        joint_state[n].acceleration = whiteNoise(1e-4);
    }
    joint_state.time = base::Time::now();

    return joint_state;
}

base::samples::RigidBodyStateSE3 randomFloatingBaseState(const base::RigidBodyStateSE3 &initial_state){
    base::samples::RigidBodyStateSE3 floating_base_state;
    floating_base_state.pose = initial_state.pose;
    for(int i = 0; i < 3; i++){
        floating_base_state.pose.position[i] += whiteNoise(1e-4);
        floating_base_state.twist.linear[i] = whiteNoise(1e-4);
        floating_base_state.twist.angular[i] = whiteNoise(1e-4);
        floating_base_state.acceleration.linear[i] = whiteNoise(1e-4);
        floating_base_state.acceleration.angular[i] = whiteNoise(1e-4);
    }
    floating_base_state.time = base::Time::now();
    return floating_base_state;
}

base::samples::RigidBodyStateSE3 randomConstraintReference(){
    base::samples::RigidBodyStateSE3 ref;
    for(int i = 0; i < 3; i++){
        ref.twist.linear[i] = whiteNoise(1e-4);
        ref.twist.angular[i] = whiteNoise(1e-4);
        ref.acceleration.linear[i] = whiteNoise(1e-4);
        ref.acceleration.angular[i] = whiteNoise(1e-4);
    }
    ref.time = base::Time::now();
    return ref;
}

void toCSV(map<string, base::VectorXd> res, const string &filename){
    std::fstream myfile;
    myfile.open(filename, ios_base::out | ios_base::app);
    if(!myfile.is_open())
        throw std::runtime_error("Error opening file " + filename);

    if(myfile.tellg() == 0){
        for(auto it : res)
            myfile << it.first << " ";
        myfile << "\n";
    }

    for(int i = 0; i < res.begin()->second.size(); i++){
        for(auto it : res)
             myfile << res[it.first][i] << " ";
        myfile << "\n";
    }
}

}
