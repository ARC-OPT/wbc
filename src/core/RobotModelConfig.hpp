#ifndef WBC_CORE_ROBOTMODELCONFIG_HPP
#define WBC_CORE_ROBOTMODELCONFIG_HPP

#include <vector>
#include <string>

namespace wbc{

/**
 * @brief Describes a rigid contact between a robot link and the environment. This can be either a point or surface contact
 */
class Contact{
public:
    Contact(){

    }
    Contact(std::string frame_id, int active, double mu) : frame_id(frame_id), active(active), mu(mu){

    }
    Contact(std::string frame_id, int active, double mu, double wx, double wy) : frame_id(frame_id), active(active), mu(mu), wx(wx), wy(wy){

    }
    std::string frame_id; /** ID of the link in contact */
    int active;           /** Is the link with ID frame_id currently in contact?*/
    double mu;            /** Friction coeffcient*/
    double wx;            /** x-dimension of the contact surface (only for surface contacts)*/
    double wy;            /** y-dimension of the contact surface (only for surface contacts)*/
};

/**
 * @brief Robot Model configuration class, containts information like urdf file, type of robot model to be loaded, etc.
 */
struct RobotModelConfig{
public:
    RobotModelConfig(){
        floating_base = false;
    }
    RobotModelConfig(const std::string& file_or_string,
                     const bool floating_base = false,
                     const std::vector<Contact> &contact_points = std::vector<Contact>(),
                     const std::string& submechanism_file = "",
                     const std::vector<std::string> &joint_blacklist = std::vector<std::string>()) :
        file_or_string(file_or_string),
        submechanism_file(submechanism_file),
        floating_base(floating_base),
        contact_points(contact_points),
        joint_blacklist(joint_blacklist){

    }

    /** Absolute path to URDF file describing the robot model or URDF string.*/
    std::string file_or_string;
    /** Only Hyrodyn models: Absolute path to submechanism file, which describes the kinematic structure including parallel mechanisms.*/
    std::string submechanism_file;
    /** Does the robot have a floating base? Attaches a virtual 6 DoF floating base to the model, default is false*/
    bool floating_base;
    /** Optional: Link names that are possibly in contact with the environment. These have to be valid link names in the robot model.*/
    std::vector<Contact> contact_points;
    /** Optional: List of joints, which shall be ignored by the WBC, i.e., these joints will be set to "fixed" joints in the loaded urdf model*/
    std::vector<std::string> joint_blacklist;
};

}
#endif // WBC_CORE_ROBOTMODELCONFIG_HPP
