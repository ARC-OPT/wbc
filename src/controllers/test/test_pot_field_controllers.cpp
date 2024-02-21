#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MAIN
#include <boost/test/unit_test.hpp>
#include <fstream>
#include <iostream>

#include "../CartesianPotentialFieldsController.hpp"
#include "../RadialPotentialField.hpp"
#include "../PlanarPotentialField.hpp"
#include "../JointLimitAvoidanceController.hpp"

using namespace std;
using namespace ctrl_lib;

void runPotentialFieldController(std::string filename,
                                 CartesianPotentialFieldsController* ctrl,
                                 base::samples::RigidBodyStateSE3 start_pos,
                                 double cycleTime){

    FILE* fp = fopen(filename.c_str(), "w");

    /*cout << "Running potential field controller: " << endl << endl;
    cout << "Maximum Ctrl. Out. is " << ctrl->getMaxControlOutput().transpose()<< endl;
    cout << "Maximum influence distance is ";
    for(size_t i = 0; i < ctrl->getFields().size(); i++)
        cout << ctrl->getFields()[i]->influence_distance << " ";
    cout<<endl;
    cout << "Prop. Gain is " << ctrl->getPGain().transpose() << endl;
    sleep(1);*/

    base::samples::RigidBodyStateSE3 feedback = start_pos, control_output;
    std::vector<PotentialFieldPtr> fields = ctrl->getFields();

    for(uint i = 0; i < 100000; i++)
    {
        control_output = ctrl->update(feedback);

        for(uint j = 0; j < ctrl->getDimension(); j++) fprintf(fp, "%f ", feedback.pose.position(j));
        for(uint j = 0; j < fields.size(); j++)
            for(uint k = 0; k < fields[j]->dimension; k++)
                fprintf(fp, "%f ", fields[j]->pot_field_center(k));
        for(uint j = 0; j < ctrl->getDimension(); j++) fprintf(fp, "%f ", control_output.twist.linear(j));
        fprintf(fp, "\n");

        feedback.pose.position += control_output.twist.linear * cycleTime;

    }
    fclose(fp);
    /*cout << "Results have been saved in " << filename << "  in following order: x x0 y" << endl;
    cout << "  where: x      = Current position" << endl;
    cout << "         x0     = Potential field origin" << endl;
    cout << "         y      = Control output" << endl << endl;*/
}

int plot(std::string gnuplot_command){
    FILE* fp = fopen("gnuplotFile", "w");
    fprintf(fp,"%s\n", gnuplot_command.c_str());
    fclose(fp);
    return system("gnuplot -persist gnuplotFile");
}

BOOST_AUTO_TEST_CASE(radial_field)
{
    const double dim = 3;
    const double cycleTime = 0.01;
    const double propGain = 10;
    const double influence_distance = 10;

    PotentialFieldPtr field = std::make_shared<RadialPotentialField>(dim, "radial_field");
    field->influence_distance = influence_distance;
    field->pot_field_center << 0,0,0;

    std::vector<PotentialFieldPtr> fields;
    fields.push_back(field);

    base::VectorXd p_gain;
    p_gain.setConstant(dim, propGain);

    CartesianPotentialFieldsController controller;
    controller.setFields(fields);
    controller.setPGain(p_gain);

    BOOST_CHECK(controller.getPGain() == p_gain);
    BOOST_CHECK(controller.getMaxControlOutput() == base::Vector3d::Constant(std::numeric_limits<double>::max()));
    BOOST_CHECK(controller.getDimension() == 3);
    BOOST_CHECK(controller.getFields()[0]->pot_field_center == base::Vector3d::Zero());
    BOOST_CHECK(controller.getFields()[0]->influence_distance == influence_distance);
    BOOST_CHECK(controller.getFields()[0]->dimension == 3);
    BOOST_CHECK(controller.getFields()[0]->name == "radial_field");

    base::samples::RigidBodyStateSE3 start_pos;
    start_pos.pose.position << 1,0,0;
    runPotentialFieldController("tmp.txt", &controller, start_pos, cycleTime);

    BOOST_CHECK(controller.getFields()[0]->distance.norm() >= influence_distance);

    // Install gnuplot and uncomment to plot right away
    /*std::string cmd = "plot 'tmp.txt' using 1 with lines title 'Current position (x)', "
                      "'tmp.txt' using 4 with lines title 'Pot. Field center (x0)', "
                      "'tmp.txt' using 7 with lines title 'ctrl. out (y)'";
    plot(cmd);*/
}

BOOST_AUTO_TEST_CASE(constrained_radial_field)
{
    const uint dim = 3;
    const double cycleTime = 0.01;
    const double propGain = 10;
    const double influence_distance = 10;
    const double max_control_output = 0.2;

    PotentialFieldPtr field = std::make_shared<RadialPotentialField>(dim, "constrained_radial_field");
    field->influence_distance = influence_distance;
    field->pot_field_center<< 0,0,0;

    std::vector<PotentialFieldPtr> fields;
    fields.push_back(field);

    base::VectorXd p_gain, max_ctrl_out;
    p_gain.setConstant(dim, propGain);
    max_ctrl_out.setConstant(dim, max_control_output);

    CartesianPotentialFieldsController controller;
    controller.setFields(fields);
    controller.setPGain(p_gain);
    controller.setMaxControlOutput(max_ctrl_out);

    BOOST_CHECK(controller.getPGain() == p_gain);
    BOOST_CHECK(controller.getMaxControlOutput() == max_ctrl_out);
    BOOST_CHECK(controller.getDimension() == 3);
    BOOST_CHECK(controller.getFields()[0]->pot_field_center == base::Vector3d::Zero());
    BOOST_CHECK(controller.getFields()[0]->influence_distance == influence_distance);
    BOOST_CHECK(controller.getFields()[0]->dimension == 3);
    BOOST_CHECK(controller.getFields()[0]->name == "constrained_radial_field");

    base::samples::RigidBodyStateSE3 start_pos;
    start_pos.pose.position << 1,0,0;
    runPotentialFieldController("tmp.txt", &controller, start_pos, cycleTime);

    BOOST_CHECK(controller.getFields()[0]->distance.norm() >= influence_distance);

    // Install gnuplot and uncomment to plot right away
    /*std::string cmd = "plot 'tmp.txt' using 1 with lines title 'Current position (x)',"
                      "'tmp.txt' using 4 with lines title 'Pot. Field center (x0)', "
                      "'tmp.txt' using 7 with lines title 'ctrl. out (y)'";
    plot(cmd);*/
}

BOOST_AUTO_TEST_CASE(planar_field){

    const int dim = 3;
    const double cycleTime = 0.01;
    const double influence_distance = 3.0;
    const double propGain = 0.1;
    const double max_control_output = 0.5;

    PlanarPotentialFieldPtr field = std::make_shared<PlanarPotentialField>("planar_field");
    field->influence_distance = influence_distance;
    field->n.resize(3);
    field->pot_field_center << 0,0,0;
    field->n << 0,1,0;

    std::vector<PotentialFieldPtr> fields;
    fields.push_back(field);

    base::VectorXd p_gain, max_ctrl_out;
    p_gain.setConstant(dim, propGain);
    max_ctrl_out.setConstant(dim, max_control_output);

    CartesianPotentialFieldsController controller;
    controller.setFields(fields);
    controller.setPGain(p_gain);
    controller.setMaxControlOutput(max_ctrl_out);

    BOOST_CHECK(controller.getPGain() == p_gain);
    BOOST_CHECK(controller.getMaxControlOutput() == max_ctrl_out);
    BOOST_CHECK(controller.getDimension() == 3);
    BOOST_CHECK(controller.getFields()[0]->pot_field_center == base::Vector3d::Zero());
    BOOST_CHECK(controller.getFields()[0]->influence_distance == influence_distance);
    BOOST_CHECK(controller.getFields()[0]->dimension == 3);
    BOOST_CHECK(controller.getFields()[0]->name == "planar_field");

    base::samples::RigidBodyStateSE3 start_pos;
    start_pos.pose.position << 0,0.2,0;

    runPotentialFieldController("tmp.txt", &controller, start_pos, cycleTime);

    BOOST_CHECK(controller.getFields()[0]->distance.norm() >= influence_distance);

    // Install gnuplot and uncomment to plot right away
    /*std::string cmd = "plot 'tmp.txt' using 1 with lines title 'Current x-position',"
                      "'tmp.txt' using 2 with lines title 'Current y-Position'";
    plot(cmd);*/
}

BOOST_AUTO_TEST_CASE(multi_radial_field)
{
    const uint dim = 3;
    const double cycleTime = 0.01;
    const double propGain = 0.1;
    const double maxCtrlOut = 0.1;

    PotentialFieldPtr field1 = std::make_shared<RadialPotentialField>(dim, "radial_field_1");
    field1->influence_distance = 0.3;
    field1->pot_field_center << 0, 0, 0;
    PotentialFieldPtr field2 = std::make_shared<RadialPotentialField>(dim, "radial_field_2");
    field2->influence_distance = 3;
    field2->pot_field_center << 2, 0.2, 0;

    std::vector<PotentialFieldPtr> fields;
    fields.push_back(field1);
    fields.push_back(field2);

    base::VectorXd p_gain, max_ctrl_out;
    p_gain.setConstant(dim, propGain);
    max_ctrl_out.setConstant(dim, maxCtrlOut);

    CartesianPotentialFieldsController controller;
    controller.setFields(fields);
    controller.setPGain(p_gain);
    controller.setMaxControlOutput(max_ctrl_out);

    BOOST_CHECK(controller.getPGain() == p_gain);
    BOOST_CHECK(controller.getMaxControlOutput() == max_ctrl_out);
    BOOST_CHECK(controller.getDimension() == 3);
    BOOST_CHECK(controller.getFields()[0]->pot_field_center == base::Vector3d(0, 0, 0));
    BOOST_CHECK(controller.getFields()[0]->influence_distance == field1->influence_distance);
    BOOST_CHECK(controller.getFields()[0]->dimension == 3);
    BOOST_CHECK(controller.getFields()[0]->name == "radial_field_1");
    BOOST_CHECK(controller.getFields()[1]->pot_field_center == base::Vector3d(2, 0.2, 0));
    BOOST_CHECK(controller.getFields()[1]->influence_distance == field2->influence_distance);
    BOOST_CHECK(controller.getFields()[1]->dimension == 3);
    BOOST_CHECK(controller.getFields()[1]->name == "radial_field_2");

    base::samples::RigidBodyStateSE3 start_pos;
    start_pos.pose.position << 0.0, 0.1, 0.0;
    runPotentialFieldController("tmp.txt", &controller, start_pos, cycleTime);

    BOOST_CHECK(controller.getFields()[0]->distance.norm() >= field1->influence_distance);
    BOOST_CHECK(controller.getFields()[1]->distance.norm() >= field2->influence_distance);

    /*std::string cmd = "plot 'tmp.txt' using 1:2 with lines title 'Current position',"
                      "'tmp.txt' using 4:5 title 'Pot. Field 1 center', "
                      "'tmp.txt' using 7:8 title 'Pot. Field 2 center'";

    // Install gnuplot and uncomment to plot right away
    plot(cmd);*/
}


BOOST_AUTO_TEST_CASE(joint_limit_avoidance)
{
    const uint dim = 2;
    const double cycleTime = 0.01;
    const double propGain = 0.1;
    const double maxCtrlOut = 0.1;
    const double influence_dist = 0.1;

    base::VectorXd p_gain, max_ctrl_out;
    p_gain.setConstant(dim, propGain);
    max_ctrl_out.setConstant(dim, maxCtrlOut);

    base::JointLimits limits;
    base::JointLimitRange range_1;
    range_1.max.position = 1.0;
    range_1.min.position = -1.0;
    base::JointLimitRange range_2;
    range_2.max.position = 0.5;
    range_2.min.position = -0.5;
    limits.elements.push_back(range_1);
    limits.elements.push_back(range_2);
    limits.names.push_back("joint_1");
    limits.names.push_back("joint_2");

    base::VectorXd influence_distance(limits.size());
    influence_distance.setConstant(influence_dist);

    JointLimitAvoidanceController controller(limits, influence_distance);
    controller.setPGain(p_gain);
    controller.setMaxControlOutput(max_ctrl_out);

    BOOST_CHECK(controller.getPGain() == p_gain);
    BOOST_CHECK(controller.getMaxControlOutput() == max_ctrl_out);
    BOOST_CHECK(controller.getDimension() == dim);

    base::samples::Joints feedback;
    feedback.names.push_back("joint_1");
    feedback.names.push_back("joint_2");
    base::JointState pos_1;
    pos_1.position = 0.95;
    base::JointState pos_2;
    pos_2.position = -0.45;
    feedback.elements.push_back(pos_1);
    feedback.elements.push_back(pos_2);

    double dt = 0.01;
    base::commands::Joints control_out;
    int n = 0;
    while(n++<100){
        BOOST_CHECK_NO_THROW(control_out = controller.update(feedback));

        feedback[0].position += control_out[0].speed * dt;
        feedback[1].position += control_out[1].speed * dt;

       /* printf("Current position:  %.4f %.4f\n", feedback[0].position, feedback[1].position);
        printf("Control output:    %.4f %.4f\n", control_out[0].speed, control_out[1].speed);
        printf("Pot. field center: %4f %.4f\n", controller.getFields()[0]->pot_field_center(0), controller.getFields()[1]->pot_field_center(0));
        printf(".........................................\n");*/
        usleep(dt*1000*1000);
    }
}
