#include "Wbc.hpp"
#include "constraints/Constraint.hpp"

namespace wbc{

Wbc::Wbc(){

}

Wbc::~Wbc(){

}

Constraint* Wbc::getConstraint(const std::string& name){

    for(size_t i = 0; i < constraints.size(); i++){
        for(size_t j = 0; j < constraints[i].size(); j++){
            if(constraints[i][j]->config.name == name)
                return constraints[i][j];
        }
    }
    LOG_ERROR("No such constraint: %s", name.c_str());
    throw std::invalid_argument("Invalid constraint name");
}

std::vector<std::string> Wbc::getTaskFrameIDs(){
    std::vector<std::string> ids;
    for(size_t i = 0; i < constraints.size(); i++)
    {
        for(size_t j = 0; j < constraints[i].size(); j++)
        {
            if(constraints[i][j]->config.type == cart)
            {
                const std::string &root = constraints[i][j]->config.root;
                if (std::find(ids.begin(), ids.end(), root) == ids.end())
                    ids.push_back(root);

                const std::string &tip = constraints[i][j]->config.tip;
                if (std::find(ids.begin(), ids.end(), tip) == ids.end())
                    ids.push_back(tip);

                const std::string &ref_frame = constraints[i][j]->config.ref_frame;
                if (std::find(ids.begin(), ids.end(), ref_frame) == ids.end())
                    ids.push_back(ref_frame);
            }
        }
    }

    return ids;
}

std::vector<int> Wbc::getNumberOfConstraintsPerPriority(){
    std::vector<int> nc_pp(constraints.size());
    for(size_t i = 0; i < constraints.size(); i++)
        nc_pp[i] = constraints[i].size();
    return nc_pp;
}

}
