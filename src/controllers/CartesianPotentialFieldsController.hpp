#ifndef CARTESIAN_POTENTIAL_FIELDS_CONTROLLER_HPP
#define CARTESIAN_POTENTIAL_FIELDS_CONTROLLER_HPP

#include "PotentialFieldsController.hpp"
#include "../types/RigidBodyState.hpp"

namespace wbc{

/**
 * @brief The PotentialFieldsController class implements a multi potential field controller in Cartesian space.
 */
class CartesianPotentialFieldsController : public PotentialFieldsController{
protected:
    types::RigidBodyState cartesian_control_output;

public:
    CartesianPotentialFieldsController();

    /**
     * @brief update Compute control output. Saturation will be applied if its has been set
     * @return control_output Control output
     */
    const types::RigidBodyState& update(const types::RigidBodyState& feedback);
};

}

#endif
