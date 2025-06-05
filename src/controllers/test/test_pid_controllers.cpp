#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MAIN
#include <boost/test/unit_test.hpp>
#include <iostream>
#include "../WrenchPIDController.hpp"
#include "../../types/Wrench.hpp"
#include <boost/assert.hpp>

using namespace std;
using namespace wbc;


BOOST_AUTO_TEST_CASE(configuration_test){
    PIDController controller(2);

    // Valid PID params
    PIDCtrlParams params_valid(2);
    BOOST_CHECK_NO_THROW(controller.setPID(params_valid));
}

BOOST_AUTO_TEST_CASE(wrench_pid_controller_test){

    PIDCtrlParams params(6);
    params.p_gain.setConstant(10);

    Eigen::VectorXd max_ctrl_out(6), dead_zone(6);
    max_ctrl_out.setConstant(5.0);
    dead_zone.setConstant(0.0);

    WrenchPIDController controller;
    controller.setPID(params);
    controller.setDeadZone(dead_zone);
    controller.setMaxCtrlOutput(max_ctrl_out);

    types::Wrench feedback;
    types::Wrench setpoint;
    types::RigidBodyState control_output;
    for(int i = 0; i < 3; i++){
        setpoint.force[i]  = (double)i;
        setpoint.torque[i] = (double)i;
    }
    feedback.force.setZero();
    feedback.torque.setZero();

    double diff = 1e10;
    double dt = 0.01;

    while(diff > dead_zone.norm() + 1e-5){

        control_output = controller.update(setpoint, feedback, dt);

        /*printf("..................................................................\n");
        printf("Setpoint:          %.6f %.6f %.6f\n", setpoint.force[0], setpoint.force[1], setpoint.force[2]);
        printf("Feedback:          %.6f %.6f %.6f\n", feedback.force[0], feedback.force[1], setpoint.force[2]);
        printf("\n");
        printf("Control output:    %.6f %.6f %.6f\n", control_output.twist.linear[0], control_output.twist.linear[1], control_output.twist.linear[2]);
        printf("Max Ctrl. Output:  %.6f %.6f %.6f\n", controller.maxCtrlOutput()(0), controller.maxCtrlOutput()(1), controller.maxCtrlOutput()(2));
        printf("Dead Zone:         %.6f %.6f %.6f\n", controller.deadZone()(0), controller.deadZone()(1), controller.deadZone()(2));
        printf("P-Gain:            %.6f %.6f %.6f\n", controller.getPID().p_gain(0), controller.getPID().p_gain(1), controller.getPID().p_gain(2));
        printf("I-Gain:            %.6f %.6f %.6f\n", controller.getPID().i_gain(0), controller.getPID().i_gain(1), controller.getPID().i_gain(2));
        printf("D-Gain:            %.6f %.6f %.6f\n", controller.getPID().d_gain(0), controller.getPID().d_gain(1), controller.getPID().d_gain(2));
        printf("..................................................................\n\n");*/

        feedback.force  += control_output.twist.linear * dt;
        feedback.torque += control_output.twist.angular * dt;
        double new_diff  = (setpoint.force  - feedback.force).norm() +
                           (setpoint.torque - feedback.torque).norm();

        BOOST_CHECK(new_diff <= diff);
        diff = new_diff;

        usleep(dt*1000*1000);
    }
}
