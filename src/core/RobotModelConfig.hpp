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
    ActiveContact(int active, double mu, double wx, double wy) : active(active), mu(mu), wx(wx), wy(wy){

    }
    int active; /** In contact*/
    double mu;  /** Friction coeffcient*/
    double wx;  /** x-dimension of the contact surface (only for surface contacts)*/
    double wy;  /** y-dimension of the contact surface (only for surface contacts)*/
};

class ActiveContacts : public base::NamedVector<ActiveContact>{
public:
    uint getNumberOfActiveContacts() const{
        uint nac = 0;
        for(auto e : elements){
            if(e.active) nac++;
        }
        return nac;
    }
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
    RobotModelConfig(const std::string& file_or_string,
                     const bool floating_base = false,
                     const ActiveContacts &contact_points = ActiveContacts(),
                     const std::string& submechanism_file = "") :
        file_or_string(file_or_string),
        submechanism_file(submechanism_file),
        type("pinocchio"),
        floating_base(floating_base),
        contact_points(contact_points){

    }
    void validate(){
        if(file_or_string.empty())
            throw std::runtime_error("Invalid Robot model config. File path or string must not be empty!");
        if(type == "hyrodyn" && submechanism_file.empty())
            throw std::runtime_error("Invalid Robot model config. If you choose 'hyrodyn' as type, submechanism_file must not be empty!");
        if(floating_base && contact_points.empty())
            throw std::runtime_error("Invalid Robot model config. If floating_base is set to true, contact_points must not be empty!");
    }

    /** Absolute path to URDF file describing the robot model or URDF string.*/
    std::string file_or_string;
    /** Only Hyrodyn models: Absolute path to submechanism file, which describes the kinematic structure including parallel mechanisms.*/
    std::string submechanism_file;
    /** Model type. Must be the exact name of one of the registered robot model plugins. See src/robot_models for all available plugins. Default is pinocchio*/
    std::string type;
    /** Optional: Attach a virtual 6 DoF floating base to the model: Naming scheme of the joints is currently fixed:
      * floating_base_trans_x, floating_base_trans_y, floating_base_trans_z,
      * floating_base_rot_x, floating_base_rot_y, floating_base_rot_z*/
    bool floating_base;
    /** Optional: Link names that are possibly in contact with the environment. These have to be valid link names in the robot model.*/
    ActiveContacts contact_points;

    std::vector<std::string> joint_blacklist;
};

}
#endif // ROBOTMODELCONFIG_HPP
