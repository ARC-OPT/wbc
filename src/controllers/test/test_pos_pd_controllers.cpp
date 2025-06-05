#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MAIN
#include <boost/test/unit_test.hpp>
#include <fstream>
#include <iostream>

#include "../CartesianPosPDController.hpp"
#include "../JointPosPDController.hpp"
#include "../../types/RigidBodyState.hpp"

using namespace std;
using namespace wbc;


BOOST_AUTO_TEST_CASE(cart_pos_pd_controller){
    srand(time(NULL));
    CartesianPosPDController ctrl;
    Eigen::VectorXd p_gain(6), max_ctrl_out(6), dead_zone(6);
    p_gain.setConstant(10);
    max_ctrl_out.setConstant(5);
    dead_zone.setConstant(0.01);

    ctrl.setPGain(p_gain);
    ctrl.setMaxCtrlOutput(max_ctrl_out);

    BOOST_CHECK(p_gain == ctrl.pGain());
    BOOST_CHECK(max_ctrl_out == ctrl.maxCtrlOutput());

    types::Pose ref_pose;
    ref_pose.position = Eigen::Vector3d((double)rand() / RAND_MAX,
                                        (double)rand() / RAND_MAX,
                                         (double)rand() / RAND_MAX);
    ref_pose.orientation = Eigen::AngleAxisd((double)rand() / RAND_MAX, Eigen::Vector3d::Unit(0)) *
                           Eigen::AngleAxisd((double)rand() / RAND_MAX, Eigen::Vector3d::Unit(1)) *
                           Eigen::AngleAxisd((double)rand() / RAND_MAX, Eigen::Vector3d::Unit(2));
    types::Twist ref_twist;
    ref_twist.setZero();

    types::Pose pose;
    pose.position.setZero();
    pose.orientation.setIdentity();

    double diff = 1e10;
    double dt = 0.01;

    Eigen::Vector3d euler = pose.orientation.toRotationMatrix().eulerAngles(2, 1, 0);
    while(diff > dead_zone.norm() + 1e-3){
        types::Twist control_out;

        control_out = ctrl.update(ref_pose, ref_twist, pose);

        pose.position += control_out.linear * dt;
        euler += control_out.angular * dt;
        pose.orientation = Eigen::AngleAxisd(euler(2), Eigen::Vector3d::Unit(2)) *
                           Eigen::AngleAxisd(euler(1), Eigen::Vector3d::Unit(1)) *
                           Eigen::AngleAxisd(euler(0), Eigen::Vector3d::Unit(0));

        /*printf("..................................................................\n");
        printf("Time:                 %s\n", control_out.time.toString().c_str());
        printf("\n");
        printf("Setpoint position:    x: %.6f,   y: %.6f,  z: %.6f\n", setpoint.pose.position(0), setpoint.pose.position(1), setpoint.pose.position(2));
        printf("Feedback position:    x: %.6f,   y: %.6f,  z: %.6f\n", feedback.pose.position(0), feedback.pose.position(1), feedback.pose.position(2));
        printf("Setpoint orientation: qx: %.6f, qy: %.6f, qz: %.6f, qw: %.6f\n", setpoint.pose.orientation.x(), setpoint.pose.orientation.y(),
                                                                                 setpoint.pose.orientation.z(), setpoint.pose.orientation.w());
        printf("Feedback orientation: qx: %.6f, qy: %.6f, qz: %.6f, qw: %.6f\n", feedback.pose.orientation.x(), feedback.pose.orientation.y(),
                                                                                 feedback.pose.orientation.z(), feedback.pose.orientation.w());
        printf("\n");
        printf("Control output twist: %.6f %.6f %.6f %.6f %.6f %.6f\n", control_out.twist.linear(0), control_out.twist.linear(1), control_out.twist.linear(2),
                                                                        control_out.twist.angular(0),control_out.twist.angular(1),control_out.twist.angular(2));
        printf("Max Control Output:   %.6f %.6f %.6f %.6f %.6f %.6f\n", ctrl.maxCtrlOutput()(0), ctrl.maxCtrlOutput()(1), ctrl.maxCtrlOutput()(2),
                                                                        ctrl.maxCtrlOutput()(3), ctrl.maxCtrlOutput()(4), ctrl.maxCtrlOutput()(5));
        printf("Dead Zone:            %.6f %.6f %.6f %.6f %.6f %.6f\n", ctrl.deadZone()(0), ctrl.deadZone()(1), ctrl.deadZone()(2),
                                                                        ctrl.deadZone()(3), ctrl.deadZone()(4), ctrl.deadZone()(5));
        printf("P-Gain:               %.6f %.6f %.6f %.6f %.6f %.6f\n", ctrl.pGain()(0), ctrl.pGain()(1), ctrl.pGain()(2),
                                                                        ctrl.pGain()(3), ctrl.pGain()(4), ctrl.pGain()(5));
        printf("D-Gain:               %.6f %.6f %.6f %.6f %.6f %.6f\n", ctrl.dGain()(0), ctrl.dGain()(1), ctrl.dGain()(2),
                                                                        ctrl.dGain()(3), ctrl.dGain()(4), ctrl.dGain()(5));
        printf("..................................................................\n\n");*/

        Eigen::VectorXd tw(6);
        tw << (ref_pose - pose).linear, (ref_pose - pose).angular;
        double new_diff = tw.norm();

        BOOST_CHECK(new_diff <= diff);

        diff = new_diff;
        usleep(dt*1000*1000);
    }
}

BOOST_AUTO_TEST_CASE(jnt_pos_pd_controller){
    srand(time(NULL));
    JointPosPDController ctrl(2);
    Eigen::Vector2d p_gain, d_gain, max_ctrl_out, dead_zone;
    p_gain.setConstant(10);
    d_gain.setConstant(0);
    max_ctrl_out.setConstant(5);
    dead_zone.setConstant(0.01);

    BOOST_CHECK_NO_THROW(ctrl.setPGain(p_gain));
    BOOST_CHECK_NO_THROW(ctrl.setDGain(d_gain));
    BOOST_CHECK_NO_THROW(ctrl.setMaxCtrlOutput(max_ctrl_out));

    BOOST_CHECK(p_gain == ctrl.pGain());
    BOOST_CHECK(d_gain == ctrl.dGain());
    BOOST_CHECK(max_ctrl_out == ctrl.maxCtrlOutput());

    Eigen::VectorXd ref_pos(2), ref_vel(2), pos(2);
    ref_pos[0] = (double)rand() / RAND_MAX;
    ref_pos[1] = (double)rand() / RAND_MAX;
    ref_vel[0] = 0;
    ref_vel[1] = 0;

    pos[0] = (double)rand() / RAND_MAX;
    pos[1] = (double)rand() / RAND_MAX;

    double diff = 1e10;
    double dt = 0.01;

    while(diff > dead_zone.norm() + 1e-3){
        Eigen::VectorXd control_out;

        BOOST_CHECK_NO_THROW(control_out = ctrl.update(ref_pos, ref_vel, pos));

        /*printf("..................................................................\n");
        printf("Setpoint:          %.6f %.6f\n", setpoint.position[0], setpoint.position[1]);
        printf("Feedback:          %.6f %.6f\n", feedback.position[0], feedback.position[1]);
        printf("\n");
        printf("Control output:    %.6f %.6f\n", control_out.velocity[0], control_out.velocity[1]);
        printf("Max Ctrl. Output:  %.6f %.6f\n", ctrl.maxCtrlOutput()(0), ctrl.maxCtrlOutput()(1));
        printf("Dead Zone:         %.6f %.6f\n", ctrl.deadZone()(0), ctrl.deadZone()(1));
        printf("P-Gain:            %.6f %.6f\n", ctrl.pGain()(0), ctrl.pGain()(1));
        printf("D-Gain:            %.6f %.6f\n", ctrl.dGain()(0), ctrl.dGain()(1));
        printf("..................................................................\n\n");*/
        pos += control_out * dt;

        double new_diff  = (ref_pos-pos).norm();

        BOOST_CHECK(new_diff <= diff);

        diff = new_diff;
        usleep(dt*1000*1000);
    }
}
