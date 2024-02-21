#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MAIN
#include <boost/test/unit_test.hpp>
#include <fstream>
#include <iostream>

#include "../CartesianPosPDController.hpp"
#include "../JointPosPDController.hpp"
#include "../ControllerTools.hpp"
#include <base/samples/RigidBodyStateSE3.hpp>

using namespace std;
using namespace ctrl_lib;


BOOST_AUTO_TEST_CASE(configuration_test){
    PosPDController controller(2);

    // Invalid P-Gain
    BOOST_CHECK_THROW(controller.setPGain(base::VectorXd(1)), std::runtime_error);

    // Invalid D-Gain
    BOOST_CHECK_THROW(controller.setDGain(base::VectorXd(1)), std::runtime_error);

    // Invalid FF-Gain
    BOOST_CHECK_THROW(controller.setFFGain(base::VectorXd(1)), std::runtime_error);

    // Invalid Dead zone
    BOOST_CHECK_THROW(controller.setDeadZone(base::VectorXd(1)), std::runtime_error);

    // Invalid Saturation
    BOOST_CHECK_THROW(controller.setMaxCtrlOutput(base::VectorXd(1)), std::runtime_error);
}

BOOST_AUTO_TEST_CASE(cart_pos_pd_controller){
    srand(time(NULL));
    CartesianPosPDController ctrl;
    base::Vector6d p_gain, d_gain, max_ctrl_out, dead_zone;
    p_gain.setConstant(10);
    d_gain.setConstant(0);
    max_ctrl_out.setConstant(5);
    dead_zone.setConstant(0.01);

    BOOST_CHECK_NO_THROW(ctrl.setPGain(p_gain));
    BOOST_CHECK_NO_THROW(ctrl.setDGain(d_gain));
    BOOST_CHECK_NO_THROW(ctrl.setMaxCtrlOutput(max_ctrl_out));
    BOOST_CHECK_NO_THROW(ctrl.setDeadZone(dead_zone));

    BOOST_CHECK(p_gain == ctrl.pGain());
    BOOST_CHECK(d_gain == ctrl.dGain());
    BOOST_CHECK(max_ctrl_out == ctrl.maxCtrlOutput());
    BOOST_CHECK(dead_zone == ctrl.deadZone());

    base::samples::RigidBodyStateSE3 setpoint;
    setpoint.pose.position = base::Vector3d((double)rand() / RAND_MAX,
                                            (double)rand() / RAND_MAX,
                                            (double)rand() / RAND_MAX);
    setpoint.pose.orientation = Eigen::AngleAxisd((double)rand() / RAND_MAX, Eigen::Vector3d::Unit(0)) *
                                Eigen::AngleAxisd((double)rand() / RAND_MAX, Eigen::Vector3d::Unit(1)) *
                                Eigen::AngleAxisd((double)rand() / RAND_MAX, Eigen::Vector3d::Unit(2));

    base::samples::RigidBodyStateSE3 feedback;
    feedback.pose.position.setZero();
    feedback.pose.orientation.setIdentity();

    double diff = 1e10;
    double dt = 0.01;

    base::Vector3d euler = base::getEuler(feedback.pose.orientation);
    while(diff > dead_zone.norm() + 1e-3){
        base::samples::RigidBodyStateSE3 control_out;

        BOOST_CHECK_NO_THROW(control_out = ctrl.update(setpoint, feedback));

        feedback.pose.position += control_out.twist.linear * dt;
        euler += control_out.twist.angular * dt;
        feedback.pose.orientation = Eigen::AngleAxisd(euler(2), Eigen::Vector3d::Unit(2)) *
                                    Eigen::AngleAxisd(euler(1), Eigen::Vector3d::Unit(1)) *
                                    Eigen::AngleAxisd(euler(0), Eigen::Vector3d::Unit(0));

        base::Vector3d euler_target = base::getEuler(setpoint.pose.orientation);

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

        base::Vector6d tw;
        tw << (setpoint.pose - feedback.pose).linear, (setpoint.pose - feedback.pose).angular;
        double new_diff = tw.norm();

        BOOST_CHECK(new_diff <= diff);

        diff = new_diff;
        usleep(dt*1000*1000);
    }
}

BOOST_AUTO_TEST_CASE(jnt_pos_pd_controller){
    srand(time(NULL));
    std::vector<std::string> joint_names;
    joint_names.push_back("joint_a");
    joint_names.push_back("joint_b");
    JointPosPDController ctrl(joint_names);
    base::Vector2d p_gain, d_gain, max_ctrl_out, dead_zone;
    p_gain.setConstant(10);
    d_gain.setConstant(0);
    max_ctrl_out.setConstant(5);
    dead_zone.setConstant(0.01);

    BOOST_CHECK_NO_THROW(ctrl.setPGain(p_gain));
    BOOST_CHECK_NO_THROW(ctrl.setDGain(d_gain));
    BOOST_CHECK_NO_THROW(ctrl.setMaxCtrlOutput(max_ctrl_out));
    BOOST_CHECK_NO_THROW(ctrl.setDeadZone(dead_zone));

    BOOST_CHECK(p_gain == ctrl.pGain());
    BOOST_CHECK(d_gain == ctrl.dGain());
    BOOST_CHECK(max_ctrl_out == ctrl.maxCtrlOutput());
    BOOST_CHECK(dead_zone == ctrl.deadZone());

    base::commands::Joints setpoint;
    setpoint.resize(2);
    setpoint.names = joint_names;
    setpoint[0].position = (double)rand() / RAND_MAX;
    setpoint[1].position = (double)rand() / RAND_MAX;

    base::samples::Joints feedback;
    feedback.resize(2);
    feedback.names = joint_names;
    feedback[0].position = (double)rand() / RAND_MAX;
    feedback[1].position = (double)rand() / RAND_MAX;

    double diff = 1e10;
    double dt = 0.01;

    while(diff > dead_zone.norm() + 1e-3){
        base::commands::Joints control_out;

        BOOST_CHECK_NO_THROW(control_out = ctrl.update(setpoint, feedback));

        /*printf("..................................................................\n");
        printf("Time:              %s\n", control_out.time.toString().c_str());
        printf("\n");
        printf("Setpoint:          %.6f %.6f\n", setpoint[0].position, setpoint[1].position);
        printf("Feedback:          %.6f %.6f\n", feedback[0].position, feedback[1].position);
        printf("\n");
        printf("Control output:    %.6f %.6f\n", control_out[0].speed, control_out[1].speed);
        printf("Max Ctrl. Output:  %.6f %.6f\n", ctrl.maxCtrlOutput()(0), ctrl.maxCtrlOutput()(1));
        printf("Dead Zone:         %.6f %.6f\n", ctrl.deadZone()(0), ctrl.deadZone()(1));
        printf("P-Gain:            %.6f %.6f\n", ctrl.pGain()(0), ctrl.pGain()(1));
        printf("D-Gain:            %.6f %.6f\n", ctrl.dGain()(0), ctrl.dGain()(1));
        printf("..................................................................\n\n");*/
        for(int i = 0; i < control_out.size(); i++)
            feedback[i].position += control_out[i].speed * dt;

        double new_diff  = ctrl.getControlError().norm();

        BOOST_CHECK(new_diff <= diff);

        diff = new_diff;
        usleep(dt*1000*1000);
    }
}
