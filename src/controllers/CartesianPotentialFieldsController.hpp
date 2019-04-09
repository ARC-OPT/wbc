#ifndef CARTESIAN_POTENTIAL_FIELDS_CONTROLLER_HPP
#define CARTESIAN_POTENTIAL_FIELDS_CONTROLLER_HPP

#include "PotentialFieldsController.hpp"
#include "../types/CartesianState.hpp"

namespace ctrl_lib{

/**
 * @brief The PotentialFieldsController class implements a multi potential field controller in Cartesian space.
 */
class CartesianPotentialFieldsController : public PotentialFieldsController{
protected:
    wbc::CartesianState cartesian_control_output;

public:
    CartesianPotentialFieldsController();

    /**
     * @brief update Compute control output. Saturation will be applied if its has been set
     * @return control_output Control output
     */
    const wbc::CartesianState& update(const wbc::CartesianState& feedback);
};

}

#endif
