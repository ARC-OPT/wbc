#ifndef WBC_PY_CONTROLLERS_HPP
#define WBC_PY_CONTROLLERS_HPP

#include "../eigen_conversion.h"
#include "../base_types_conversion.h"
#include "../std_vector_conversion.h"
#include "controllers/CartesianPosPDController.hpp"
#include "controllers/JointPosPDController.hpp"

namespace wbc_py {

class JointPosPDController : public ctrl_lib::JointPosPDController{
public:
    JointPosPDController(const std::vector<std::string> &joint_names);
    base::NamedVector<base::JointState> update(const base::NamedVector<base::JointState> &setpoint,
                                               const base::NamedVector<base::JointState> &feedback);
};
}

#endif
