#pragma once

#include "core/RobotModelConfig.hpp"

namespace wbc_py{
wbc::ActiveContacts toActiveContacts(const base::NamedVector<wbc::ActiveContact> &contacts_in){
    wbc::ActiveContacts contacts_out;
    contacts_out.elements = contacts_in.elements;
    contacts_out.names = contacts_in.names;
    return contacts_out;
}

base::NamedVector<wbc::ActiveContact> fromActiveContacts(const wbc::ActiveContacts& contacts_in){
    base::NamedVector<wbc::ActiveContact> contacts_out;
    contacts_out.names = contacts_in.names;
    contacts_out.elements = contacts_in.elements;
    return contacts_out;
}

class RobotModelConfig : public wbc::RobotModelConfig{
public:
    base::NamedVector<wbc::ActiveContact> getActiveContacts(){
        return fromActiveContacts(contact_points);
    }
    void setActiveContacts(base::NamedVector<wbc::ActiveContact> contacts_in){
        contact_points = toActiveContacts(contacts_in);
    }
};

wbc::RobotModelConfig toRobotModelConfig(RobotModelConfig cfg_in){
    RobotModelConfig cfg = cfg_in;
    return cfg;
}

RobotModelConfig fromRobotModelConfig(wbc::RobotModelConfig cfg_in){
    RobotModelConfig cfg;
    cfg.file_or_string = cfg_in.file_or_string;
    cfg.submechanism_file = cfg_in.submechanism_file;
    cfg.floating_base = cfg_in.floating_base;
    cfg.contact_points = cfg_in.contact_points;
    return cfg;
}
}
