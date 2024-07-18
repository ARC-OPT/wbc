#ifndef URDF_TOOLS_HPP
#define URDF_TOOLS_HPP

#include <string>
#include <vector>
#include <urdf_parser/urdf_parser.h>
#include "../types/JointLimits.hpp"

namespace wbc {

class URDFTools{
public:
    /** Return all non-fixed joints from the given URDF file*/
    static std::vector<std::string> jointNamesFromURDF(const urdf::ModelInterfaceSharedPtr& urdf_model);

   /** Return limits for all non-fixed joints from the given URDF model*/
    static void jointLimitsFromURDF(const urdf::ModelInterfaceSharedPtr& urdf_model, types::JointLimits& limits, const std::vector<std::string>& joint_names);

    /** Return Root link from given URDF file*/
    static const std::string rootLinkFromURDF(const urdf::ModelInterfaceSharedPtr& urdf_model);

    /** Print whole URDF tree*/
    static void printTree(urdf::LinkConstSharedPtr link, int level = 0);

    /** Set all blacklisted joints in robot model to fixed*/
    static bool applyJointBlacklist(urdf::ModelInterfaceSharedPtr& robot_urdf, const std::vector<std::string> &blacklist);
};

}

#endif
