#include <boost/test/unit_test.hpp>
#include <iostream>

#include "controllers/JointTorquePIDController.hpp"
#include <ctrl_types/CartesianState.hpp>

using namespace std;
using namespace ctrl_lib;

BOOST_AUTO_TEST_CASE(joint_torque_pid_controller){

    std::vector<std::string> joint_names;
    joint_names.push_back("joint_a");
    joint_names.push_back("joint_b");

    PIDCtrlParams params(joint_names.size());
    params.p_gain.setConstant(1);

    base::Vector2d max_ctrl_out, dead_zone;
    max_ctrl_out.setConstant(0.5);
    dead_zone.setConstant(0.01);

    JointTorquePIDController controller(joint_names);
    controller.setPID(params);
    controller.setDeadZone(dead_zone);
    controller.setMaxCtrlOutput(max_ctrl_out);

    base::samples::Joints feedback;
    base::commands::Joints setpoint, control_output;
    feedback.resize(joint_names.size());
    setpoint.resize(joint_names.size());
    feedback.names = setpoint.names = joint_names;
    setpoint[0].effort = 1;
    setpoint[1].effort = 2;
    feedback[0].effort = 0;
    feedback[1].effort = 0;

    double diff = 1e10;
    double dt = 0.01;

    while(diff > dead_zone.norm() + 1e-5){

        BOOST_CHECK_NO_THROW(control_output = controller.update(setpoint, feedback, dt));

        printf("..................................................................\n");
        printf("Time:              %s\n", control_output.time.toString().c_str());
        printf("\n");
        printf("Setpoint:          %.6f %.6f\n", setpoint[0].effort, setpoint[1].effort);
        printf("Feedback:          %.6f %.6f\n", feedback[0].effort, feedback[1].effort);
        printf("\n");
        printf("Control output:    %.6f %.6f\n", control_output[0].speed, control_output[1].speed);
        printf("Max Ctrl. Output:  %.6f %.6f\n", controller.maxCtrlOutput()(0), controller.maxCtrlOutput()(1));
        printf("Dead Zone:         %.6f %.6f\n", controller.deadZone()(0), controller.deadZone()(1));
        printf("P-Gain:            %.6f %.6f\n", controller.getPID().p_gain(0), controller.getPID().p_gain(1));
        printf("I-Gain:            %.6f %.6f\n", controller.getPID().i_gain(0), controller.getPID().i_gain(1));
        printf("D-Gain:            %.6f %.6f\n", controller.getPID().d_gain(0), controller.getPID().d_gain(1));
        printf("..................................................................\n\n");

        for(int i = 0; i < control_output.size(); i++)
            feedback[i].effort += control_output[i].speed * dt;
        double new_diff  = sqrt(pow(setpoint[0].effort - feedback[0].effort,2) +
                                pow(setpoint[1].effort - feedback[1].effort,2));

        BOOST_CHECK(new_diff <= diff);

        diff = new_diff;

        usleep(dt*1000*1000);
    }
}

