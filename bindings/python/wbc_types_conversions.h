#pragma once

#include "core/RobotModelConfig.hpp"

namespace wbc_py{

wbc::ActiveContacts toActiveContacts(const base::NamedVector<int> &contacts_in){
    wbc::ActiveContacts contacts_out;
    contacts_out.elements = contacts_in.elements;
    contacts_out.names = contacts_in.names;
    return contacts_out;
}

base::NamedVector<int> fromActiveContacts(const wbc::ActiveContacts& contacts_in){
    base::NamedVector<int> contacts_out;
    contacts_out.names = contacts_in.names;
    contacts_out.elements = contacts_in.elements;
    return contacts_out;
}
}
