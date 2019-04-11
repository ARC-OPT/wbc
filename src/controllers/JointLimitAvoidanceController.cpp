#include "JointLimitAvoidanceController.hpp"
#include "RadialPotentialField.hpp"
#include <base/samples/Joints.hpp>

using namespace std;

namespace ctrl_lib {

JointLimitAvoidanceController::JointLimitAvoidanceController(const base::JointLimits& limits,
                                                             const base::VectorXd &influence_distance) :
    PotentialFieldsController(limits.size()){

    if(limits.size() != influence_distance.size())
        throw runtime_error("Size of joint limits and influence distance vector must be the same");

    fields.clear();

    for(size_t i = 0; i < limits.size(); i++){

        PotentialFieldPtr field = make_shared<RadialPotentialField>(1, limits.names[i]);
        field->pot_field_center.setConstant(1, limits[i].max.position);
        field->influence_distance = influence_distance(i);
        fields.push_back(field);
    }

    joint_limits = limits;
    joints_control_output.resize(limits.size());
    joints_control_output.names = limits.names;

    epsilon = 1e-9;
}

const base::commands::Joints& JointLimitAvoidanceController::update(const base::samples::Joints& feedback){

    for(size_t i = 0; i < joint_limits.size(); i++){

        const string& name = joint_limits.names[i];
        double min = joint_limits[i].min.position;
        double max = joint_limits[i].max.position;
        double pos = feedback[name].position;

        // Prevent infinite control action:
        if(pos >= max)
            pos = max - epsilon;
        if(pos <= min)
            pos = min  + epsilon;

        // Set potential field center position depending on which is closer: upper or lower limit
        double c_pos = fabs(max - pos)  < fabs(pos - min) ? max : min;
        fields[i]->pot_field_center.setConstant(1, c_pos);

        // Compute Control output
        base::VectorXd pos_vect(1);
        pos_vect << pos;
        fields[i]->update(pos_vect);
        control_output(i) = p_gain(i) * fields[i]->gradient(0);
    }

    // Apply saturation
    applySaturation(control_output, control_output);

    // Convert to joints data types
    joints_control_output.time = base::Time::now();
    for(size_t i = 0; i < dimension; i++)
        joints_control_output[i].speed = control_output(i);

    return joints_control_output;
}
}
