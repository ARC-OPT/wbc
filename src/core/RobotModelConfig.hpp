#ifndef ROBOTMODELCONFIG_HPP
#define ROBOTMODELCONFIG_HPP

#include <base/samples/RigidBodyStateSE3.hpp>
#include <base/NamedVector.hpp>

namespace wbc{

class ActiveContact{
public:
    ActiveContact(){

    }
    ActiveContact(int active, double mu) : active(active), mu(mu){

    }
    int active; /** In contact*/
    double mu;  /** Friction coeffcient*/
    double wx;  /** x-dimension of the contact surface (only for surface contacts)*/
    double wy;  /** y-dimension of the contact surface (only for surface contacts)*/
};

class ActiveContacts : public base::NamedVector<ActiveContact>{
};

/**
 * @brief Robot Model configuration class
 */
struct RobotModelConfig{
public:
    RobotModelConfig(){
        floating_base = false;
        type = "pinocchio";
    }
    RobotModelConfig(const std::string& file,
                     const bool floating_base = false,
                     const ActiveContacts &contact_points = ActiveContacts(),
                     const std::string& submechanism_file = "") :
        file(file),
        submechanism_file(submechanism_file),
        type("pinocchio"),
        floating_base(floating_base),
        contact_points(contact_points){

    }

    /** Absolute path to URDF file describing the robot model.*/
    std::string file;
    /** Only Hyrodyn models: Absolute path to submechanism file, which describes the kinematic structure including parallel mechanisms.*/
    std::string submechanism_file;
    /** Model type. Must be the exact name of one of the registered robot model plugins. See src/robot_models for all available plugins. Default is pinocchio*/
    std::string type;
    /** Optional: Attach a virtual 6 DoF floating base to the model: Naming scheme of the joints is currently fixed:
      * floating_base_trans_x, floating_base_trans_y, floating_base_trans_z,
      * floating_base_rot_x, floating_base_rot_y, floating_base_rot_z*/
    bool floating_base;
    /** Optional: Link names that are possibly in contact with the environment. These have to valid link names in the robot model.*/
    ActiveContacts contact_points;
};

}
#endif // ROBOTMODELCONFIG_HPP
