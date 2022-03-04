#ifndef ACTIVATION_FUNCTION_HPP
#define ACTIVATION_FUNCTION_HPP

#include <base/Eigen.hpp>
#include <stdexcept>

namespace ctrl_lib{

enum activationType{NO_ACTIVATION,              /** Activation values will always be one */
                    STEP_ACTIVATION,            /** If input > threshold, activation will be one, else 0 */
                    LINEAR_ACTIVATION,          /** If input < threshold, activation will be increase linearly the input, otherwise 1 */
                    QUADRATIC_ACTIVATION,       /** If input < threshold, activation will be increase quadratically the input, otherwise 1 */
                    PROPORTIONAL_ACTIVATION};   /** If input < threshold, activation will be same as given value, otherwise 1 */

/** Define different activation function like linear, quadratic, etc.
 */
struct ActivationFunction{
    ActivationFunction() :
        type(NO_ACTIVATION){
    }

    /** Threshold value. Effect depends on type activation*/
    double threshold;
    activationType type;
    base::VectorXd activation;

    const base::VectorXd &compute(const base::VectorXd& values){

        if(type != NO_ACTIVATION && (threshold < 0 || threshold > 1))
            throw std::invalid_argument("ActivationFunction::compute: Threshold must be within [0..1]");

        activation.resize(values.size());
        activation.setZero();

        for(uint i = 0; i < values.size(); i++){
            switch(type){
            case NO_ACTIVATION:
                activation(i) = 1;
                break;
            case STEP_ACTIVATION:
                if(fabs(values(i)) > threshold)
                    activation(i) = 1;
                else
                    activation(i) = 0;
                break;
            case LINEAR_ACTIVATION:
                if(values(i) < threshold)
                    activation(i) = (1.0/threshold) * values(i);
                else
                    activation(i) = 1;
                break;
            case QUADRATIC_ACTIVATION:
                if(values(i) < threshold)
                    activation(i) = (1.0/(threshold*threshold)) * (values(i)*values(i));
                else
                    activation(i) = 1;
                break;
            case PROPORTIONAL_ACTIVATION:
                if(values(i) < threshold)
                    activation(i) = values(i);
                else
                    activation(i) = 1;

                break;
            default:{
                std::stringstream s;
                s << "Invalid activation type: " << type;
                throw std::invalid_argument(s.str());
            }
            }
        }
        return activation;
    }
};

}

#endif
