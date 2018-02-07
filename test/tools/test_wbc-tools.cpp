#include <boost/test/unit_test.hpp>
#include "tools/tools.hpp"
#include <kdl_conversions/KDLConversions.hpp>
#include <kdl/frames_io.hpp>

using namespace std;
using namespace wbc;

BOOST_AUTO_TEST_CASE(test_tools)
{
    cout<<"Testing Tools ..."<<endl<<endl;

    srand (time(NULL));

    base::samples::RigidBodyState pose_a, pose_b;
    for(int i = 0; i < 3; i++) pose_a.position(i) = (double)rand() / RAND_MAX;
    pose_a.orientation = Eigen::AngleAxisd((double)rand() / RAND_MAX, base::Vector3d::UnitX()) *
                         Eigen::AngleAxisd((double)rand() / RAND_MAX, base::Vector3d::UnitY()) *
                         Eigen::AngleAxisd((double)rand() / RAND_MAX, base::Vector3d::UnitZ());
    for(int i = 0; i < 3; i++) pose_b.position(i) = (double)rand() / RAND_MAX;
    pose_b.orientation = Eigen::AngleAxisd((double)rand() / RAND_MAX, base::Vector3d::UnitX()) *
                         Eigen::AngleAxisd((double)rand() / RAND_MAX, base::Vector3d::UnitY()) *
                         Eigen::AngleAxisd((double)rand() / RAND_MAX, base::Vector3d::UnitZ());

    base::Vector6d diff;
    pose_diff(pose_a, pose_b, 1, diff);

    cout<<"RigidBodyState A is "<<endl;
    cout<<"Pos: "<<pose_a.position.transpose()<<endl;
    cout<<"Ori: "<<pose_a.orientation.toRotationMatrix()<<endl;
    cout<<"RigidBodyState B is "<<endl;
    cout<<"Pos: "<<pose_b.position.transpose()<<endl;
    cout<<"Ori: "<<pose_b.orientation.toRotationMatrix()<<endl;
    cout<<"Diff is: "<<diff.transpose()<<endl<<endl;

    KDL::Frame pose_a_kdl, pose_b_kdl;
    kdl_conversions::RigidBodyState2KDL(pose_a, pose_a_kdl);
    kdl_conversions::RigidBodyState2KDL(pose_b, pose_b_kdl);

    KDL::Twist diff_kdl = KDL::diff(pose_a_kdl, pose_b_kdl);

    for(int i = 0; i < 6; i++)
        BOOST_CHECK(fabs(diff(i)-diff_kdl(i)) < 1e-10);

    cout<<"Checking with KDL..."<<endl;
    cout<<"KDL Frame A is "<<endl;
    cout<<pose_a_kdl<<endl;
    cout<<"KDL Frame B is "<<endl;
    cout<<pose_b_kdl<<endl;
    cout<<"Diff is: "<<diff_kdl<<endl<<endl;

    cout<<"...done"<<endl;
}
