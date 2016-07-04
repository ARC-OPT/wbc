#include <kdl/frames_io.hpp>
#include <boost/test/unit_test.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include "../src/WbcVelocity.hpp"
#include "../src/solvers/HierarchicalLeastSquaresSolver.hpp"
#include "../src/robot_models/KinematicRobotModelKDL.hpp"
#include "../src/common/OptProblem.hpp"
#include "../src/robot_models/RobotModelConfig.hpp"
#include "../src/common/TaskFrame.hpp"

using namespace std;
using namespace wbc;

/**
 * Test hierarchical solver with random input data
 */

BOOST_AUTO_TEST_CASE(hierarchical_solver)
{
    srand (time(NULL));

    const uint NO_JOINTS = 3;
    const uint NO_CONSTRAINTS = 2;
    const double NORM_MAX = 5.75;
    const double MIN_EIGENVALUE = 1e-9;

    HierarchicalLeastSquaresSolver solver;
    std::vector<int> ny_per_prio(1,NO_CONSTRAINTS);

    BOOST_CHECK(solver.configure(ny_per_prio, NO_JOINTS) == true);

    BOOST_NOEXCEPT(solver.setMaxSolverOutputNorm(NORM_MAX));
    BOOST_NOEXCEPT(solver.setMinEigenvalue(MIN_EIGENVALUE));

    std::vector<OptProblem*> input;
    WeightedLSSimple prio_0;
    prio_0.resize(NO_CONSTRAINTS, NO_JOINTS);

    for(uint i = 0; i < NO_CONSTRAINTS*NO_JOINTS; i++ )
        prio_0.A.data()[i] = (rand()%1000)/1000.0;
    for(uint i = 0; i < NO_CONSTRAINTS; i++ )
        prio_0.y_ref.data()[i] = (rand()%1000)/1000.0;
    prio_0.W.setConstant(1);

    input.push_back(&prio_0);

    cout<<"\n----------------------- Testing Hierarchical Solver ----------------------"<<endl<<endl;
    cout<<"Number of priorities: "<<ny_per_prio.size()<<endl;
    cout<<"Constraints per priority: "; for(uint i = 0; i < ny_per_prio.size(); i++) cout<<ny_per_prio[i]<<" "; cout<<endl;
    cout<<"No of joints: "<<NO_JOINTS<<endl;
    cout<<"\n----------------------- Solver Input ----------------------"<<endl<<endl;
    for(uint i = 0; i < ny_per_prio.size(); i++){
        cout<<"Priority: "<<i<<endl;
        cout<<"Constraint Matrix: "<<endl; cout<<prio_0.A<<endl;
        cout<<"Reference: "<<endl; cout<<prio_0.y_ref<<endl;
        cout<<endl;
    }

    Eigen::VectorXd solver_output;
    try{
        solver.solve(input, solver_output);
    }
    catch(std::exception e){
        BOOST_ERROR("Solver.solve threw an exception");
    }

    cout<<"---------------------- Solver Output ------------------------"<<endl;
    cout<<"q_dot = "<<endl;
    cout<<solver_output<<endl;
    cout<<"---------------------- TEST ------------------------"<<endl<<endl;
    for(uint i = 0; i < ny_per_prio.size(); i++){
        cout<<" ---- Priority: "<<i<<" ----"<<endl<<endl;
        Eigen::VectorXd test = prio_0.A*solver_output;
        cout<<"A * q_dot: "<<endl;
        cout<<test<<endl; cout<<endl;
        for(uint j = 0; j < NO_CONSTRAINTS; j++)
            BOOST_CHECK(fabs(test(j) - prio_0.y_ref(j)) < 1e-9);
    }

    cout<<"\n............................."<<endl;
}

/**
 * Test Kinematic KDL model
 */
BOOST_AUTO_TEST_CASE(kinematic_model){

    KinematicRobotModelKDL model;
    std::string camera_frame = "kuka_lbr_top_left_camera";

    std::string robot_urdf_path  = std::getenv("AUTOPROJ_CURRENT_ROOT") + std::string("/control/wbc/test/kuka_lbr.urdf");
    RobotModelConfig robot_model(robot_urdf_path);

    std::string object_urdf_path = std::getenv("AUTOPROJ_CURRENT_ROOT") + std::string("/control/wbc/test/object.urdf");
    base::samples::RigidBodyState initial_pose;
    initial_pose.setTransform(Eigen::Affine3d::Identity());
    RobotModelConfig object_model(object_urdf_path, camera_frame, initial_pose);

    std::cout<<"Loading robot model ..."<<std::endl<<std::endl;
    BOOST_CHECK(model.loadModel(robot_model) == true);
    std::cout<<"Attaching object model to " << camera_frame <<"..."<<std::endl<<std::endl;
    BOOST_CHECK(model.loadModel(object_model) == true);
    BOOST_CHECK(model.hasTaskFrame("object") == false);
    BOOST_CHECK(model.addTaskFrame("object") == true);

    TaskFrame *tf = model.getTaskFrame("object");

    base::samples::Joints joint_state;
    joint_state.resize(tf->joint_names.size());
    joint_state.names = tf->joint_names;
    for(uint i = 0; i < tf->joint_names.size(); i++)
        joint_state[i].position = 0;
    model.update(joint_state);

    std::cout<<"Object Pose in robot base frame: "<<std::endl;
    std::cout<<"Position: "<<std::endl;
    std::cout<<tf->pose.position<<std::endl;
    std::cout<<"Orientation(qx,qy,qz,qw): "<<std::endl;
    std::cout<<tf->pose.orientation.x()<<""<<tf->pose.orientation.y()<<" "<<tf->pose.orientation.z()<<" "<<tf->pose.orientation.w()<<std::endl<<std::endl;

    base::samples::RigidBodyState old_pose = tf->pose;

    base::samples::RigidBodyState segment_pose;
    segment_pose.sourceFrame = "object";
    segment_pose.position.setZero();
    segment_pose.position(2) = 0.5;
    segment_pose.orientation.setIdentity();
    std::vector<base::samples::RigidBodyState> poses;
    poses.push_back(segment_pose);

    std::cout<<"Moving object (in camera coordinates) by:"<<std::endl;
    std::cout<<segment_pose.position <<std::endl<<std::endl;

    model.update(joint_state, poses);

    std::cout<<"New Object Pose in robot base: "<<std::endl;
    std::cout<<"Position: "<<std::endl;
    std::cout<<tf->pose.position<<std::endl;
    std::cout<<"Orientation(qx,qy,qz,qw): "<<std::endl;
    std::cout<<tf->pose.orientation.x()<<" "<<tf->pose.orientation.y()<<" "<<tf->pose.orientation.z()<<" "<<tf->pose.orientation.w()<<std::endl<<std::endl;

    std::cout<<"Distance initial pose --> new pose is: "<<std::endl;
    std::cout<<(tf->pose.position - old_pose.position).norm()<<std::endl;

    BOOST_CHECK( (segment_pose.position.norm() - (tf->pose.position - old_pose.position).norm()) < 1e-9 );

}

