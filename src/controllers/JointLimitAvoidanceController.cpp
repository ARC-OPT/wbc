#include "JointLimitAvoidanceController.hpp"
#include "RadialPotentialField.hpp"

using namespace std;

namespace wbc {

JointLimitAvoidanceController::JointLimitAvoidanceController(const types::JointLimits& limits,
                                                             const std::vector<std::string> joint_names,
                                                             const Eigen::VectorXd &influence_distance) :
    PotentialFieldsController(influence_distance.size()){

    assert(limits.max.position.size() == dimension);
    assert(limits.min.position.size() == dimension);
    assert(joint_names.size() == dimension);
    assert(influence_distance.size() == dimension);
    fields.clear();

    joint_limits = limits;
    for(size_t i = 0; i < joint_names.size(); i++){

        PotentialFieldPtr field = make_shared<RadialPotentialField>(1, joint_names[i]);
        field->pot_field_center.setConstant(1, joint_limits.max.position[i]);
        field->influence_distance = influence_distance(i);
        fields.push_back(field);
    }

    joints_control_output.resize(joint_names.size());
    epsilon = 1e-9;
}

const types::JointCommand& JointLimitAvoidanceController::update(const types::JointState& feedback){

    assert(p_gain.size() == dimension);

    for(long i = 0; i < joint_limits.min.position.size(); i++){

        double min = joint_limits.min.position[i];
        double max = joint_limits.max.position[i];
        double pos = feedback.position[i];

        // Prevent infinite control action:
        if(pos >= max)
            pos = max - epsilon;
        if(pos <= min)
            pos = min  + epsilon;

        // Set potential field center position depending on which is closer: upper or lower limit
        double c_pos = fabs(max - pos)  < fabs(pos - min) ? max : min;
        fields[i]->pot_field_center.setConstant(1, c_pos);

        // Compute Control output
        Eigen::VectorXd pos_vect(1);
        pos_vect << pos;
        fields[i]->update(pos_vect);
        control_output(i) = p_gain(i) * fields[i]->gradient(0);
    }

    // Apply saturation
    applySaturation(control_output, control_output);

    // Convert to joints data types
    for(size_t i = 0; i < dimension; i++)
        joints_control_output.acceleration[i] = joints_control_output.velocity[i] = control_output(i);

    return joints_control_output;
}
}
