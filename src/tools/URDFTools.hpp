#ifndef URDF_TOOLS_HPP
#define URDF_TOOLS_HPP

#include <string>
#include <vector>
#include <urdf_parser/urdf_parser.h>

namespace base{
class JointLimits;
}

namespace wbc {

class URDFTools{
public:
    /** Return all non-fixed joints from the given URDF file*/
    static std::vector<std::string> jointNamesFromURDF(const std::string &filename);

    /** Return all non-fixed joints from the given URDF file*/
    static std::vector<std::string> jointNamesFromURDF(const urdf::ModelInterfaceSharedPtr& urdf_model);

    /** Return limits for all non-fixed joints from the given URDF file*/
    static void jointLimitsFromURDF(const std::string& filename, base::JointLimits& limits);

    /** Return limits for all non-fixed joints from the given URDF model*/
    static void jointLimitsFromURDF(const urdf::ModelInterfaceSharedPtr& urdf_model, base::JointLimits& limits);

    /** Return Root link from given URDF file*/
    static const std::string rootLinkFromURDF(const std::string &filename);

    /** Return robot name from given URDF file*/
    static const std::string robotNameFromURDF(const std::string &filename);

    /** Print whole URDF tree*/
    static void printTree(const std::string& filename);

    /** Print whole URDF tree*/
    static void printTree(urdf::LinkConstSharedPtr link, int level = 0);

    /** Create a 6 Dof virtual floating base URDF and add it to the robot model*/
    static std::vector<std::string> addFloatingBaseToURDF(urdf::ModelInterfaceSharedPtr& robot_urdf, const std::string &world_frame_id = "world");

    /** Set all blacklisted joints in robot model to fixed*/
    static bool applyJointBlacklist(urdf::ModelInterfaceSharedPtr& robot_urdf, const std::vector<std::string> &blacklist);
};

}

#endif
