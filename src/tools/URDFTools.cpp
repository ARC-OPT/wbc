#include "URDFTools.hpp"
#include <base/JointLimits.hpp>
#include <base-logging/Logging.hpp>
#include <urdf_model/link.h>
#include <stack>
#include <fstream>

namespace wbc {

std::vector<std::string> URDFTools::jointNamesFromURDF(const std::string &filename){
    urdf::ModelInterfaceSharedPtr urdf_model = urdf::parseURDFFile(filename);
    if (!urdf_model)
        throw urdf::ParseError("Cannot load URDF from file " + filename);

    return jointNamesFromURDF(urdf_model);
}

std::vector<std::string> URDFTools::jointNamesFromURDF(const urdf::ModelInterfaceSharedPtr& urdf_model){

    std::vector<std::string> joint_names;
    std::map<std::string, urdf::LinkSharedPtr> link_map = urdf_model->links_;
    std::stack< std::shared_ptr<urdf::Link> > link_stack;
    link_stack.push (link_map[(urdf_model->getRoot()->name)]);
    std::stack<int> joint_index_stack;
    joint_index_stack.push(0);
    while (link_stack.size() > 0) {
        std::shared_ptr<urdf::Link> cur_link = link_stack.top();
        unsigned int joint_idx = joint_index_stack.top();
        if (joint_idx < cur_link->child_joints.size()) {
            std::shared_ptr<urdf::Joint> cur_joint = cur_link->child_joints[joint_idx];
            joint_index_stack.pop();
            joint_index_stack.push (joint_idx + 1);
            link_stack.push (link_map[cur_joint->child_link_name]);
            joint_index_stack.push(0);
            if(cur_joint->type != urdf::Joint::FIXED)
                joint_names.push_back(cur_joint->name);
        }
        else{
            link_stack.pop();
            joint_index_stack.pop();
        }
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
    auto *doc = urdf::exportURDF(robot_urdf);
    std::string filename = "/tmp/robot_urdf";
    doc->SaveFile(filename.c_str());
    std::ifstream file;
    file.open(filename);
    if(!file){
        std::cout << "Could not open " << filename << std::endl;
        exit(0);
    }
    std::string robot_xml_string( (std::istreambuf_iterator<char>(file) ),(std::istreambuf_iterator<char>()) );
    robot_xml_string.erase(robot_xml_string.find("</robot>"), std::string("</robot>").length());
    std::string floating_base = std::string("  <link name='" + world_frame_id + "'>\n")   +
            "    <inertial>" +
            "      <origin rpy='0 0 0' xyz='0 0 0'/>" +
            "      <mass value='0.001'/>" +
            "      <inertia ixx='0' ixy='0' ixz='0' iyy='0' iyz='0' izz='0'/>" +
            "    </inertial>" +
            "  </link>" +

            "  <link name='link_floating_base_trans_x'>"   +
            "    <inertial>" +
            "      <origin rpy='0 0 0' xyz='0 0 0'/>" +
            "      <mass value='0.001'/>" +
            "      <inertia ixx='0' ixy='0' ixz='0' iyy='0' iyz='0' izz='0'/>" +
            "    </inertial>" +
            "  </link\n>" +

            "  <link name='link_floating_base_trans_y'>"   +
            "    <inertial>" +
            "      <origin rpy='0 0 0' xyz='0 0 0'/>" +
            "      <mass value='0.001'/>" +
            "      <inertia ixx='0' ixy='0' ixz='0' iyy='0' iyz='0' izz='0'/>" +
            "    </inertial>" +
            "  </link\n>" +

            "  <link name='link_floating_base_trans_z'>"   +
            "    <inertial>" +
            "      <origin rpy='0 0 0' xyz='0 0 0'/>" +
            "      <mass value='0.001'/>" +
            "      <inertia ixx='0' ixy='0' ixz='0' iyy='0' iyz='0' izz='0'/>" +
            "    </inertial>" +
            "  </link\n>" +

            "  <link name='link_floating_base_rot_x'>"   +
            "    <inertial>" +
            "      <origin rpy='0 0 0' xyz='0 0 0'/>" +
            "      <mass value='0.001'/>" +
            "      <inertia ixx='0' ixy='0' ixz='0' iyy='0' iyz='0' izz='0'/>" +
            "    </inertial>" +
            "  </link\n>" +

            "  <link name='link_floating_base_rot_y'>"   +
            "    <inertial>" +
            "      <origin rpy='0 0 0' xyz='0 0 0'/>" +
            "      <mass value='0.001'/>" +
            "      <inertia ixx='0' ixy='0' ixz='0' iyy='0' iyz='0' izz='0'/>" +
            "    </inertial>" +
            "  </link\n>" +

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

bool URDFTools::applyJointBlacklist(urdf::ModelInterfaceSharedPtr& robot_urdf, const std::vector<std::string> &blacklist){

    for(auto name : blacklist){
        if(robot_urdf->joints_.count(name) == 0){
            LOG_ERROR_S << "Joint Blacklist contains joint " << name << " but this name is not in robot model " << std::endl;
            return false;
        }
        robot_urdf->joints_[name]->type = urdf::Joint::FIXED;
    }
    return true;
}

}
