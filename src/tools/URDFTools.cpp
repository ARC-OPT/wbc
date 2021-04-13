#include "URDFTools.hpp"
#include <base/JointLimits.hpp>
#include <base-logging/Logging.hpp>

namespace wbc {

std::vector<std::string> URDFTools::jointNamesFromURDF(const std::string &filename){
    urdf::ModelInterfaceSharedPtr urdf_model = urdf::parseURDFFile(filename);
    if (!urdf_model)
        throw urdf::ParseError("Cannot load URDF from file " + filename);

    return jointNamesFromURDF(urdf_model);
}

std::vector<std::string> URDFTools::jointNamesFromURDF(const urdf::ModelInterfaceSharedPtr& urdf_model){
    std::vector<std::string> joint_names;
    std::map<std::string, urdf::JointSharedPtr>::const_iterator it;
    for(it=urdf_model->joints_.begin(); it!=urdf_model->joints_.end(); ++it){
        const urdf::JointSharedPtr &joint = it->second;
        if(joint->type == urdf::Joint::FIXED)
            continue;
        joint_names.push_back(joint->name);
    }
    return joint_names;
}

void URDFTools::jointLimitsFromURDF(const std::string& filename, base::JointLimits& limits){
    urdf::ModelInterfaceSharedPtr urdf_model = urdf::parseURDFFile(filename);
    if (!urdf_model)
        throw urdf::ParseError("Cannot load URDF from file " + filename);
    jointLimitsFromURDF(urdf_model, limits);
}

void URDFTools::jointLimitsFromURDF(const urdf::ModelInterfaceSharedPtr& urdf_model, base::JointLimits& limits){

    std::map<std::string, urdf::JointSharedPtr>::const_iterator it;
    for(it=urdf_model->joints_.begin(); it!=urdf_model->joints_.end(); ++it){
        const urdf::JointSharedPtr &joint = it->second;
        base::JointLimitRange range;

        if(joint->limits){
            range.max.position = joint->limits->upper;
            range.min.position = joint->limits->lower;
            range.max.speed = joint->limits->velocity;
            range.min.speed = -joint->limits->velocity;
            range.max.effort = joint->limits->effort;
            range.min.effort = -joint->limits->effort;

            try{
                limits.getElementByName(it->first);
            }
            catch(base::JointLimits::InvalidName e){
                limits.names.push_back(it->first);
                limits.elements.push_back(range);

                LOG_INFO_S<<"Added joint limit for joint "<<it->first<<" Max. Pos: "<<range.max.position
                         <<" Min. Pos: "<<range.min.position<<" Max. Vel: "<<range.max.speed<<" Max. Effort: "<<range.max.effort<<std::endl;
            }
        }
    }
}

const std::string URDFTools::rootLinkFromURDF(const std::string &filename){
    urdf::ModelInterfaceSharedPtr urdf_model = urdf::parseURDFFile(filename);
    if (!urdf_model)
        throw urdf::ParseError("Cannot load URDF from file " + filename);

    return urdf_model->getRoot()->name;
}

const std::string URDFTools::robotNameFromURDF(const std::string &filename){
    urdf::ModelInterfaceSharedPtr urdf_model = urdf::parseURDFFile(filename);
    if (!urdf_model)
        throw urdf::ParseError("Cannot load URDF from file " + filename);
    return urdf_model->getName();
}

void URDFTools::printTree(urdf::LinkConstSharedPtr link, int level){
    level+=2;
    int count = 0;
    for(std::vector<urdf::LinkSharedPtr>::const_iterator child = link->child_links.begin(); child != link->child_links.end(); child++){
        if (*child){
            for(int j=0;j<level;j++) std::cout << "  "; //indent
            std::cout << "child(" << (count++)+1 << "):  " << (*child)->name  << std::endl;
            // first grandchild
            printTree(*child,level);
        }
        else{
            for(int j=0;j<level;j++) std::cout << " "; //indent
            std::cout << "root link: " << link->name << " has a null child!" << *child << std::endl;
        }
    }
}

void URDFTools::printTree(const std::string& filename){
    urdf::ModelInterfaceSharedPtr urdf_model = urdf::parseURDFFile(filename);
    if (!urdf_model)
        throw urdf::ParseError("Cannot load URDF from file " + filename);
    urdf::LinkConstSharedPtr root_link = urdf_model->getRoot();
    printTree(root_link);

}

std::vector<std::string> URDFTools::addFloatingBaseToURDF(urdf::ModelInterfaceSharedPtr& robot_urdf, const std::string &world_frame_id){

    std::vector<std::string> floating_base_names = {"floating_base_trans_x", "floating_base_trans_y", "floating_base_trans_z",
                                                    "floating_base_rot_x", "floating_base_rot_y", "floating_base_rot_z"};
    TiXmlDocument *doc = urdf::exportURDF(robot_urdf);
    TiXmlPrinter printer;
    doc->Accept(&printer);
    std::string robot_xml_string = printer.CStr();
    robot_xml_string.erase(robot_xml_string.find("</robot>"), std::string("</robot>").length());
    std::string floating_base = std::string("  <link name='" + world_frame_id + "'>\n")   +
            "    <inertial>" +
            "      <origin rpy='0 0 0' xyz='0 0 0'/>" +
            "      <mass value='0'/>" +
            "      <inertia ixx='0' ixy='0' ixz='0' iyy='0' iyz='0' izz='0'/>" +
            "    </inertial>" +
            "  </link>" +

            "  <link name='link_floating_base_trans_x'/>\n"   +
            "  <link name='link_floating_base_trans_y'/>\n"   +
            "  <link name='link_floating_base_trans_z'/>\n"   +
            "  <link name='link_floating_base_rot_x'/>\n"   +
            "  <link name='link_floating_base_rot_y'/>\n"   +

           "  <joint name='floating_base_trans_x' type='prismatic'>\n"   +
             "  <parent link='" + world_frame_id + "'/>\n"   +
             "  <child link='link_floating_base_trans_x'/>\n"   +
             "  <origin rpy='0 0 0' xyz='0 0 0'/>\n"   +
             "  <axis xyz='1 0 0'/>\n"   +
             "  <limit effort='1000' lower='-1000' upper='1000' velocity='1000'/>\n"   +
           "  </joint>\n"   +

           "  <joint name='floating_base_trans_y' type='prismatic'>\n"   +
             "  <parent link='link_floating_base_trans_x'/>\n"   +
             "  <child link='link_floating_base_trans_y'/>\n"   +
             "  <origin rpy='0 0 0' xyz='0 0 0'/>\n"   +
             "  <axis xyz='0 1 0'/>\n"   +
             "  <limit effort='1000' lower='-1000' upper='1000' velocity='1000'/>\n"   +
           "  </joint>\n"   +

           "  <joint name='floating_base_trans_z' type='prismatic'>\n"   +
             "  <parent link='link_floating_base_trans_y'/>\n"   +
             "  <child link='link_floating_base_trans_z'/>\n"   +
             "  <origin rpy='0 0 0' xyz='0 0 0'/>\n"   +
             "  <axis xyz='0 0 1'/>\n"   +
             "  <limit effort='1000' lower='-1000' upper='1000' velocity='1000'/>\n"   +
           "  </joint>\n"   +

           "  <joint name='floating_base_rot_x' type='revolute'>\n"   +
             "  <parent link='link_floating_base_trans_z'/>\n"   +
             "  <child link='link_floating_base_rot_x'/>\n"   +
             "  <origin rpy='0 0 0' xyz='0 0 0'/>\n"   +
             "  <axis xyz='1 0 0'/>\n"   +
             "  <limit effort='1000' lower='-1000' upper='1000' velocity='1000'/>\n"   +
           "  </joint>\n"   +

           "  <joint name='floating_base_rot_y' type='revolute'>\n"   +
             "  <parent link='link_floating_base_rot_x'/>\n"   +
             "  <child link='link_floating_base_rot_y'/>\n"   +
             "  <origin rpy='0 0 0' xyz='0 0 0'/>\n"   +
             "  <axis xyz='0 1 0'/>\n"   +
             "  <limit effort='1000' lower='-1000' upper='1000' velocity='1000'/>\n"   +
           "  </joint>\n"   +

           "  <joint name='floating_base_rot_z' type='revolute'>\n"   +
             "  <parent link='link_floating_base_rot_y'/>\n"   +
             "  <child link='" + robot_urdf->getRoot()->name + "'/>\n"   +
             "  <origin rpy='0 0 0' xyz='0 0 0'/>\n"   +
             "  <axis xyz='0 0 1'/>\n"   +
             "  <limit effort='1000' lower='-1000' upper='1000' velocity='1000'/>\n"   +
           "  </joint>\n";
    robot_xml_string += floating_base + "</robot>";
    robot_urdf = urdf::parseURDF(robot_xml_string);

    return floating_base_names;
}

void URDFTools::applyJointBlacklist(urdf::ModelInterfaceSharedPtr& robot_urdf, const std::vector<std::string> &blacklist){

    for(auto name : blacklist){
        if(robot_urdf->joints_.count(name) == 0){
            LOG_ERROR_S << "Joint Blacklist contains joint " << name << " but this name is not in robot model " << std::endl;
            throw std::runtime_error("Invalid joint blacklist configuration");
        }
        robot_urdf->joints_[name]->type = urdf::Joint::FIXED;
    }
}

}
