#include <boost/test/unit_test.hpp>
#include "robot_models/pinocchio/RobotModelPinocchio.hpp"
#include "robot_models/hyrodyn/RobotModelHyrodyn.hpp"
#include "core/RobotModelConfig.hpp"
#include "tools/URDFTools.hpp"
#include <random>

using namespace std;
using namespace wbc;

void compareRobotModels(RobotModelPinocchio robot_model_pinocchio, RobotModelHyrodyn robot_model_hyrodyn, const string frame_id ){

    int nj = robot_model_pinocchio.noOfActuatedJoints();

    base::VectorXd q(nj),qd(nj),qdd(nj);
    for(int i = 0; i < nj; i++){
        q(i) = (double)rand()/RAND_MAX;
        qd(i) = (double)rand()/RAND_MAX;
        qdd(i) = (double)rand()/RAND_MAX;
    }

    base::samples::Joints joint_state;
    joint_state.resize(nj);
    joint_state.names = robot_model_hyrodyn.actuatedJointNames();
    for(size_t i = 0; i < nj; i++){
        joint_state[i].position = q[i];
        joint_state[i].speed = qd[i];
        joint_state[i].acceleration = qdd[i];
    }
    joint_state.time = base::Time::now();

    base::samples::RigidBodyStateSE3 floating_base_state;
    floating_base_state.pose.position = base::Vector3d(double(rand())/RAND_MAX,double(rand())/RAND_MAX,double(rand())/RAND_MAX);
    floating_base_state.pose.orientation = Eigen::AngleAxisd(double(rand())/RAND_MAX, Eigen::Vector3d::UnitX())
                                         * Eigen::AngleAxisd(double(rand())/RAND_MAX, Eigen::Vector3d::UnitY())
                                         * Eigen::AngleAxisd(double(rand())/RAND_MAX, Eigen::Vector3d::UnitZ());
    floating_base_state.twist.linear  = base::Vector3d(double(rand())/RAND_MAX,double(rand())/RAND_MAX,double(rand())/RAND_MAX);
    floating_base_state.twist.angular = base::Vector3d(double(rand())/RAND_MAX,double(rand())/RAND_MAX,double(rand())/RAND_MAX);
    floating_base_state.acceleration.linear  = base::Vector3d(double(rand())/RAND_MAX,double(rand())/RAND_MAX,double(rand())/RAND_MAX);
    floating_base_state.acceleration.angular = base::Vector3d(double(rand())/RAND_MAX,double(rand())/RAND_MAX,double(rand())/RAND_MAX);
    floating_base_state.time = base::Time::now();

    BOOST_CHECK_NO_THROW(robot_model_hyrodyn.update(joint_state,floating_base_state));
    BOOST_CHECK_NO_THROW(robot_model_pinocchio.update(joint_state,floating_base_state));

    string base_link = robot_model_pinocchio.worldFrame();

    base::samples::RigidBodyStateSE3 rbs_hyrodyn = robot_model_hyrodyn.rigidBodyState(base_link, frame_id);
    base::MatrixXd Js_hyrodyn = robot_model_hyrodyn.spaceJacobian(base_link, frame_id);
    base::MatrixXd Jb_hyrodyn = robot_model_hyrodyn.bodyJacobian(base_link, frame_id);
    base::MatrixXd H_hyrodyn = robot_model_hyrodyn.jointSpaceInertiaMatrix();
    base::MatrixXd C_hyrodyn = robot_model_hyrodyn.biasForces();
    base::Acceleration acc_hyrodyn  = robot_model_hyrodyn.spatialAccelerationBias(base_link,frame_id);
    base::samples::RigidBodyStateSE3 com_hyrodyn = robot_model_hyrodyn.centerOfMass();
    base::MatrixXd com_jac_hyrodyn = robot_model_hyrodyn.comJacobian();

    base::samples::RigidBodyStateSE3 rbs_pinocchio = robot_model_pinocchio.rigidBodyState(base_link, frame_id);
    base::MatrixXd Js_pinocchio = robot_model_pinocchio.spaceJacobian(base_link, frame_id);
    base::MatrixXd Jb_pinocchio = robot_model_pinocchio.bodyJacobian(base_link, frame_id);
    base::MatrixXd H_pinocchio = robot_model_pinocchio.jointSpaceInertiaMatrix();
    base::MatrixXd C_pinocchio = robot_model_pinocchio.biasForces();
    base::Acceleration acc_pinocchio  = robot_model_pinocchio.spatialAccelerationBias(base_link,frame_id);
    base::samples::RigidBodyStateSE3 com_pinocchio = robot_model_pinocchio.centerOfMass();
    base::MatrixXd com_jac_pinocchio = robot_model_pinocchio.comJacobian();

    cout<<"................... Robot Model Hyrodyn ..........................."<<endl;
    cout<<"Pose"<<endl;
    cout<<rbs_hyrodyn.pose.position.transpose()<<endl;
    cout<<rbs_hyrodyn.pose.orientation.coeffs().transpose()<<endl;
    cout<<"Twist"<<endl;
    cout<<rbs_hyrodyn.twist.linear.transpose()<<endl;
    cout<<rbs_hyrodyn.twist.angular.transpose()<<endl;
    cout<<"Acceleration"<<endl;
    cout<<rbs_hyrodyn.acceleration.linear.transpose()<<endl;
    cout<<rbs_hyrodyn.acceleration.angular.transpose()<<endl;
    cout<<"Space Jacobian"<<endl;
    cout<<Js_hyrodyn<<endl;
    /*cout<<"Body Jacobian"<<endl;
    cout<<Jb_hyrodyn<<endl;
    cout<<"Spatial Acc Bias"<<endl;
    cout<<"Linear: "<<acc_pinocchio.linear.transpose()<<endl;
    cout<<"Angular: "<<acc_pinocchio.angular.transpose()<<endl;
    cout<<"Joint Space Inertia"<<endl;
    cout<<H_hyrodyn<<endl;
    cout<<"Bias Forces"<<endl;
    cout<<C_hyrodyn.transpose()<<endl<<endl;
    cout<<"CoM"<<endl;
    cout<<"Position:     "<<com_hyrodyn.pose.position.transpose()<<endl;
    cout<<"Velocity:     "<<com_hyrodyn.twist.linear.transpose()<<endl;
    cout<<"Acceleration: "<<com_hyrodyn.acceleration.linear.transpose()<<endl;
    cout<<"CoM Jacobian"<<endl;
    cout<<com_jac_hyrodyn<<endl<<endl;*/

    cout<<".....................Robot Model Pinocchio ......................."<<endl;
    cout<<"Pose"<<endl;
    cout<<rbs_pinocchio.pose.position.transpose()<<endl;
    cout<<rbs_pinocchio.pose.orientation.coeffs().transpose()<<endl;
    cout<<"Twist"<<endl;
    cout<<rbs_pinocchio.twist.linear.transpose()<<endl;
    cout<<rbs_pinocchio.twist.angular.transpose()<<endl;
    cout<<"Acceleration"<<endl;
    cout<<rbs_pinocchio.acceleration.linear.transpose()<<endl;
    cout<<rbs_pinocchio.acceleration.angular.transpose()<<endl;
    cout<<"Space Jacobian"<<endl;
    cout<<Js_pinocchio<<endl;
    /*cout<<"Body Jacobian"<<endl;
    cout<<Jb_pinocchio<<endl;
    cout<<"Spatial Acc Bias"<<endl;
    cout<<"Linear: "<<acc_hyrodyn.linear.transpose()<<endl;
    cout<<"Angular: "<<acc_hyrodyn.angular.transpose()<<endl;
    cout<<"Joint Space Inertia"<<endl;
    cout<<H_pinocchio<<endl;
    cout<<"Bias Forces"<<endl;
    cout<<C_pinocchio.transpose()<<endl<<endl;
    cout<<"CoM"<<endl;
    cout<<"Position:     "<<com_pinocchio.pose.position.transpose()<<endl;
    cout<<"Velocity:     "<<com_pinocchio.twist.linear.transpose()<<endl;
    cout<<"Acceleration: "<<com_pinocchio.acceleration.linear.transpose()<<endl;
    cout<<"CoM Jacobian"<<endl;
    cout<<com_jac_pinocchio<<endl<<endl;*/

    cout<<".........................................................."<<endl;

    for(int i = 0; i < 3; i++)
        BOOST_CHECK(fabs(rbs_hyrodyn.pose.position(i) - rbs_pinocchio.pose.position(i)) < 1e-3);
    if(rbs_pinocchio.pose.orientation.w() < 0 && rbs_hyrodyn.pose.orientation.w() > 0 ||
       rbs_pinocchio.pose.orientation.w() > 0 && rbs_hyrodyn.pose.orientation.w() < 0)
        rbs_hyrodyn.pose.orientation = base::Quaterniond(-rbs_hyrodyn.pose.orientation.w(),-rbs_hyrodyn.pose.orientation.x(),-rbs_hyrodyn.pose.orientation.y(),-rbs_hyrodyn.pose.orientation.z());
    for(int i = 0; i < 4; i++)
        BOOST_CHECK(fabs(rbs_hyrodyn.pose.orientation.coeffs()(i) - rbs_pinocchio.pose.orientation.coeffs()(i)) < 1e-3);
    for(int i = 0; i < 3; i++)
        BOOST_CHECK(fabs(rbs_hyrodyn.twist.linear(i) - rbs_pinocchio.twist.linear(i)) < 1e-3);
    for(int i = 0; i < 3; i++)
        BOOST_CHECK(fabs(rbs_hyrodyn.twist.angular(i) - rbs_pinocchio.twist.angular(i)) < 1e-3);
    for(int i = 0; i < 3; i++)
        BOOST_CHECK(fabs(rbs_hyrodyn.acceleration.linear(i) - rbs_pinocchio.acceleration.linear(i)) < 1e-3);
    for(int i = 0; i < 3; i++)
        BOOST_CHECK(fabs(rbs_hyrodyn.acceleration.angular(i) - rbs_pinocchio.acceleration.angular(i)) < 1e-3);
    for(int i = 0; i < 6; i++){
        for(int j = 0; j < nj; j++){
            uint idx = robot_model_hyrodyn.jointIndex(robot_model_pinocchio.jointNames()[j]);
            BOOST_CHECK(fabs(Js_hyrodyn(i,j) - Js_pinocchio(i,j)) < 1e-3);
        }
    }
    /*for(int i = 0; i < 6; i++){
        for(int j = 0; j < nj; j++){
            uint idx = robot_model_hyrodyn.jointIndex(robot_model_pinocchio.jointNames()[j]);
            BOOST_CHECK(fabs(Jb_hyrodyn(i,idx) - Jb_pinocchio(i,j)) < 1e-3);
        }
    }
    for(int i = 0; i < nj; i++){
        for(int j = 0; j < nj; j++){
            int row_idx = robot_model_hyrodyn.jointIndex(robot_model_pinocchio.jointNames()[i]);
            int col_idx = robot_model_hyrodyn.jointIndex(robot_model_pinocchio.jointNames()[j]);
            BOOST_CHECK(fabs(H_hyrodyn(row_idx,col_idx) - H_pinocchio(i,j)) < 1e-2);
        }
    }
    for(int i = 0; i < nj; i++){
        uint idx = robot_model_hyrodyn.jointIndex(robot_model_pinocchio.jointNames()[i]);
        BOOST_CHECK(fabs(C_hyrodyn(idx) - C_pinocchio(i)) < 1e-3);
    }
    for(int i = 0; i < 3; i++){
        BOOST_CHECK(fabs(acc_hyrodyn.linear(i) - acc_pinocchio.linear(i)) < 1e-3);
        BOOST_CHECK(fabs(acc_hyrodyn.angular(i) - acc_pinocchio.angular(i)) < 1e-3);
    }
    for(int i = 0; i < 3; i++){
        BOOST_CHECK(fabs(com_hyrodyn.pose.position(i) - com_pinocchio.pose.position(i)) < 1e-3);
        BOOST_CHECK(fabs(com_hyrodyn.twist.linear(i) - com_pinocchio.twist.linear(i)) < 1e-3);
        BOOST_CHECK(fabs(com_hyrodyn.acceleration.linear(i) - com_pinocchio.acceleration.linear(i)) < 1e-3);
    }
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < nj; j++){
            uint idx = robot_model_hyrodyn.jointIndex(robot_model_pinocchio.jointNames()[j]);
            BOOST_CHECK(fabs(com_jac_hyrodyn(i,idx) - com_jac_pinocchio(i,j)) < 1e-3);
        }
    }*/
}

BOOST_AUTO_TEST_CASE(compare_hyrodyn_vs_pinocchio){

    /**
     * Compare kinematics and dynamics of KDL-based robot model with Pinocchio-based robot model
     */
    srand(time(NULL));


    const string frame_id = "kuka_lbr_l_tcp";

    RobotModelPinocchio robot_model_pinocchio;
    RobotModelConfig config("../../../../models/kuka/urdf/kuka_iiwa.urdf");
    BOOST_CHECK(robot_model_pinocchio.configure(config) == true);

    RobotModelHyrodyn robot_model_hyrodyn;
    config = RobotModelConfig("../../../../models/kuka/urdf/kuka_iiwa.urdf");
    config.submechanism_file = "../../../../models/kuka/hyrodyn/kuka_iiwa.yml";
    BOOST_CHECK(robot_model_hyrodyn.configure(config) == true);

    compareRobotModels(robot_model_pinocchio, robot_model_hyrodyn, frame_id);
}

BOOST_AUTO_TEST_CASE(compare_hyrodyn_vs_pinocchio_floating_base){

    /**
     * Compare kinematics and dynamics of KDL-based robot model with Pinocchio-based robot model including floating base
     */
    srand(time(NULL));

    const string frame_id = "kuka_lbr_l_link_7";

    RobotModelPinocchio robot_model_pinocchio;
    RobotModelConfig config("../../../../models/kuka/urdf/kuka_iiwa.urdf");
    config.floating_base = true;
    BOOST_CHECK(robot_model_pinocchio.configure(config) == true);

    RobotModelHyrodyn robot_model_hyrodyn;
    config = RobotModelConfig("../../../../models/kuka/urdf/kuka_iiwa.urdf");
    config.submechanism_file = "../../../../models/kuka/hyrodyn/kuka_iiwa_floating_base.yml";
    config.floating_base = true;
    BOOST_CHECK(robot_model_hyrodyn.configure(config) == true);

    compareRobotModels(robot_model_pinocchio, robot_model_hyrodyn, frame_id);
}

