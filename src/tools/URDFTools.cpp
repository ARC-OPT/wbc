#include "URDFTools.hpp"
#include <urdf_model/link.h>
#include <stack>
#include "../tools/Logger.hpp"

namespace wbc {

using namespace types;

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

void URDFTools::jointLimitsFromURDF(const urdf::ModelInterfaceSharedPtr& urdf_model, JointLimits& limits, const std::vector<std::string>& joint_names){

    // TODO: This assumes that all non-fixed joints have joint limits
    std::map<std::string, urdf::JointSharedPtr>::const_iterator it;
    limits.resize(joint_names.size());
    for(size_t i = 0; i < joint_names.size(); i++){
        std::string n = joint_names[i];
        assert(urdf_model->joints_.count(n) == 1);
        const urdf::JointSharedPtr &joint = urdf_model->joints_[n];
        assert(joint->limits);
        assert(joint->type != urdf::Joint::FIXED);
        if(joint->limits){
            limits.max.position[i]       = joint->limits->upper;
            limits.min.position[i]       = joint->limits->lower;
            limits.max.velocity[i]       = joint->limits->velocity;
            limits.min.velocity[i]       = -joint->limits->velocity;
            limits.max.effort[i]         = joint->limits->effort;
            limits.min.effort[i]         = -joint->limits->effort;
            limits.max.acceleration[i]   = 1e4;
            limits.min.acceleration[i]   = -1e4;
        }
    }
}

const std::string URDFTools::rootLinkFromURDF(const urdf::ModelInterfaceSharedPtr& urdf_model){
    return urdf_model->getRoot()->name;
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

bool URDFTools::applyJointBlacklist(urdf::ModelInterfaceSharedPtr& robot_urdf, const std::vector<std::string> &blacklist){

    for(auto name : blacklist){
        if(robot_urdf->joints_.count(name) == 0){
            log(logERROR)<<"Blacklist contains joint "<<name<<" but this joint does not exist in robot URDF";
            return false;
        }
        robot_urdf->joints_[name]->type = urdf::Joint::FIXED;
    }
    return true;
}

}
