#include "benchmarks_common.hpp"
#include <fstream>
#include <unistd.h>

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

base::samples::Joints randomJointState(base::samples::Joints joint_state_in){
    base::samples::Joints joint_state;
    joint_state.resize(joint_state_in.size());
    joint_state.names = joint_state_in.names;
    for(auto n : joint_state.names){
        joint_state[n].position = joint_state_in[n].position + whiteNoise(1e-4);//double(rand())/RAND_MAX)* (limits[n].max.position - limits[n].min.position) + limits[n].min.position;
        joint_state[n].speed = whiteNoise(1e-4);
        joint_state[n].acceleration = whiteNoise(1e-4);
    }
    joint_state.time = base::Time::now();

    return joint_state;
}

base::samples::Joints randomJointState(base::JointLimits limits){
    base::samples::Joints joint_state;
    joint_state.resize(limits.size());
    joint_state.names = limits.names;
    for(auto n : joint_state.names){
        joint_state[n].position = (double(rand())/RAND_MAX)* (limits[n].max.position - limits[n].min.position) + limits[n].min.position;
        joint_state[n].speed = whiteNoise(1e-4);
        joint_state[n].acceleration = whiteNoise(1e-4);
    }
    joint_state.time = base::Time::now();

    return joint_state;
}

base::samples::RigidBodyStateSE3 randomFloatingBaseState(){
    base::samples::RigidBodyStateSE3 floating_base_state;
    floating_base_state.pose.position = base::Vector3d(double(rand())/RAND_MAX,double(rand())/RAND_MAX,double(rand())/RAND_MAX);
    floating_base_state.pose.orientation = Eigen::AngleAxisd(double(rand())/RAND_MAX, Eigen::Vector3d::UnitX())
                                         * Eigen::AngleAxisd(double(rand())/RAND_MAX, Eigen::Vector3d::UnitY())
                                         * Eigen::AngleAxisd(double(rand())/RAND_MAX, Eigen::Vector3d::UnitZ());
    floating_base_state.twist.linear = base::Vector3d(double(rand())/RAND_MAX,double(rand())/RAND_MAX,double(rand())/RAND_MAX);
    floating_base_state.twist.angular = base::Vector3d(double(rand())/RAND_MAX,double(rand())/RAND_MAX,double(rand())/RAND_MAX);
    floating_base_state.acceleration.linear = base::Vector3d(double(rand())/RAND_MAX,double(rand())/RAND_MAX,double(rand())/RAND_MAX);
    floating_base_state.acceleration.angular = base::Vector3d(double(rand())/RAND_MAX,double(rand())/RAND_MAX,double(rand())/RAND_MAX);
    floating_base_state.time = base::Time::now();
    floating_base_state.frame_id = "world";
    return floating_base_state;
}


base::samples::RigidBodyStateSE3 randomFloatingBaseState(base::samples::RigidBodyStateSE3 floating_base_state_in){
    base::samples::RigidBodyStateSE3 floating_base_state;
    base::Vector3d euler = base::getEuler(floating_base_state_in.pose.orientation);
    floating_base_state.pose.position = base::Vector3d(whiteNoise(1e-4),
                                                       whiteNoise(1e-4),
                                                       whiteNoise(1e-4));
    floating_base_state.pose.orientation = Eigen::AngleAxisd(whiteNoise(1e-4), Eigen::Vector3d::UnitX())
                                         * Eigen::AngleAxisd(whiteNoise(1e-4), Eigen::Vector3d::UnitY())
                                         * Eigen::AngleAxisd(whiteNoise(1e-4), Eigen::Vector3d::UnitZ());
    floating_base_state.twist.linear = base::Vector3d(whiteNoise(1e-4),
                                                      whiteNoise(1e-4),
                                                      whiteNoise(1e-4));
    floating_base_state.twist.angular = base::Vector3d(whiteNoise(1e-4),
                                                      whiteNoise(1e-4),
                                                      whiteNoise(1e-4));
    floating_base_state.acceleration.linear = base::Vector3d( whiteNoise(1e-4),
                                                      whiteNoise(1e-4),
                                                      whiteNoise(1e-4));
    floating_base_state.acceleration.angular = base::Vector3d(whiteNoise(1e-4),
                                                      whiteNoise(1e-4),
                                                      whiteNoise(1e-4));
    floating_base_state.time = base::Time::now();
    floating_base_state.frame_id = "world";
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


map<string, base::VectorXd> evaluateWBCSceneRandom(WbcScenePtr scene, int n_samples){

    base::VectorXd time_scene_update(n_samples);
    base::VectorXd time_scene_solve(n_samples);
    for(int i = 0; i < n_samples; i++){
        for(auto w : scene->getWbcConfig())
            scene->setReference(w.name, randomConstraintReference());

        RobotModelPtr robot_model = scene->getRobotModel();
        base::samples::Joints joint_state = randomJointState(robot_model->jointState(robot_model->independentJointNames()));
        base::samples::RigidBodyStateSE3 floating_base_state = randomFloatingBaseState(scene->getRobotModel()->floatingBaseState());
        scene->getRobotModel()->update(joint_state, floating_base_state);

        base::Time start = base::Time::now();
        HierarchicalQP qp = scene->update();
        time_scene_update[i] = (double)(base::Time::now()-start).toMicroseconds()/1000;

            start = base::Time::now();
            scene->solve(qp);
            time_scene_solve[i] = (double)(base::Time::now()-start).toMicroseconds()/1000;

        usleep(0.01*1e6);
    }

    map<string, base::VectorXd> results;
    results["scene_update"] = time_scene_update;
    results["scene_solve"]  = time_scene_solve;
    return results;
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
