#ifndef POTENTIAL_FIELDS_CONTROLLER_HPP
#define POTENTIAL_FIELDS_CONTROLLER_HPP

#include "PotentialFieldInfo.hpp"
#include <vector>

namespace wbc{

/**
 * @brief Eigen class for potential field controllers
 */
class PotentialFieldsController{
protected:
    Eigen::VectorXd p_gain, max_ctrl_output, control_output;
    std::vector<PotentialFieldPtr> fields;
    std::vector<PotentialFieldInfo> field_infos;
    uint dimension;

public:
    PotentialFieldsController(const uint _dimension);
    /**
     * @brief Apply Saturation on the control output. If one or more values of <in> are bigger than the
     *        Corrresponding entry of <max>, all values will be scaled down according to the biggest
     *        ratio eta = in_i / max,i
     * @param in Input vector. Size has to be same as max.
     * @param out Output vector. Will be resized if out.size() != in.size()
     */
    void applySaturation(const Eigen::VectorXd& in, Eigen::VectorXd &out);
    /** Provide new potential fields. Dimension of each field has to be the same as dimension of the controller. */
    void setFields(const std::vector<PotentialFieldPtr>& _fields);
    /** Return potential field infos*/
    std::vector<PotentialFieldInfo> getFieldInfos();
    /** Erase all potential fields*/
    void clearFields(){fields.clear();}
    /** Set proportional gain*/
    void setPGain(const Eigen::VectorXd& gain);
    /** Set maximum control output*/
    void setMaxControlOutput(const Eigen::VectorXd& new_max_control_output);
    /** Return max contorl output vector*/
    Eigen::VectorXd getMaxControlOutput(){return max_ctrl_output;}
    /** Return p_gain vector*/
    Eigen::VectorXd getPGain(){return p_gain;}
    /** Return current potential field vector*/
    const std::vector<PotentialFieldPtr> &getFields(){return fields;}
    /** return dimensionality of controller*/
    uint getDimension(){return dimension;}
};

}

#endif


