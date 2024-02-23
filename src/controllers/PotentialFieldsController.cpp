#include "PotentialFieldsController.hpp"
#include <stdexcept>
#include <limits>

using namespace wbc;
using namespace std;

PotentialFieldsController::PotentialFieldsController(const uint _dimension)
    : dimension(_dimension){

    // Make max control output infinite by default
    max_ctrl_output.setConstant(dimension, numeric_limits<double>::max());
    control_output.resize(dimension);
}

void PotentialFieldsController::setFields(const vector<PotentialFieldPtr>& _fields){
    for(PotentialFieldPtr f :  _fields){
        if(f->dimension != dimension)
            throw runtime_error("PotentialFieldsController::setFields: Dimension of controller is " + to_string(dimension) +
                                " but dimension of field with name '" + f->name + "' is " + to_string(f->dimension));
        if(!base::isnotnan(f->pot_field_center))
            throw runtime_error("PotentialFieldsController::setFields: Field with name '" + f->name + "' does not have a valid pot. field center");
    }
    fields = _fields;
}

void PotentialFieldsController::applySaturation(const base::VectorXd& in, base::VectorXd &out){
    if(in.size() != max_ctrl_output.size())
        throw invalid_argument("Controller::applySaturation: Input vector size must be same as controller dimension");
    out.resize(max_ctrl_output.size());

    //Apply saturation. Scale all values according to the maximum output
    double eta = 1;
    for(uint i = 0; i < in.size(); i++)
        eta = min( eta, max_ctrl_output(i)/fabs(in(i)) );
    out = eta * in;
}

void PotentialFieldsController::setPGain(const base::VectorXd& gain){
    if(gain.size() != dimension)
        throw runtime_error("Size of PGain vector is " + to_string(gain.size()) + " but should be " + to_string(dimension));
    p_gain = gain;
}

void PotentialFieldsController::setMaxControlOutput(const base::VectorXd& max){
    if(max.size() != dimension)
        throw runtime_error("Size of Max. Ctrl Output vector is " + to_string(max.size()) + " but should be " + to_string(dimension));
    max_ctrl_output = max;
}

vector<PotentialFieldInfo> PotentialFieldsController::getFieldInfos(){
    field_infos.resize(fields.size());
    for(size_t i = 0; i < fields.size(); i++)
        field_infos[i].fromPotentialField(fields[i]);
    return field_infos;
}
